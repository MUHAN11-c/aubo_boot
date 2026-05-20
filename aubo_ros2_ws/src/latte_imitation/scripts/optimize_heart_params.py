#!/usr/bin/env python3
"""
心形拉花参数贝叶斯优化 — Optuna TPE 采样器 喵~

优化目标: 心形轨迹几何质量评分 (最大化 0-100)

使用:
  # 模拟模式 (纯几何评分, 不需要机器人)
  python3 scripts/optimize_heart_params.py --sim --trials 50

  # 真机模式 (通过 ROS2 service 执行 + 相机评分)
  python3 scripts/optimize_heart_params.py --real --trials 30

  # 从已有 study 继续优化
  python3 scripts/optimize_heart_params.py --sim --study-name heart_v1 --trials 20
"""

import os, sys, argparse, json, time
import numpy as np
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

# 静默 optuna 日志
os.environ["OPTUNA_WARNINGS"] = "0"
import optuna
optuna.logging.set_verbosity(optuna.logging.WARNING)

from latte_imitation.heart_scorer import score_heart_trajectory, score_against_reference
from latte_imitation.latte_art import (
    LatteArtTrajectory, CupConfig, PourConfig,
    compose_full_trajectory, apply_anti_sloshing, parametric_to_cartesian,
)
from latte_imitation.promp_learner import ProMP3D
from latte_imitation.dmp_learner import DMP3D
from scipy.signal import savgol_filter

CONFIG_DIR = os.path.join(PKG_DIR, "config")
HEART_DIR = os.path.join(PKG_DIR, "resource", "heart")
DT = 0.05


def extract_forming_ref():
    """加载 TOP5 集成参考轨迹 (用于 sim 模式评分)"""
    path = os.path.join(HEART_DIR, "heart_ensemble_forming.npz")
    if os.path.exists(path):
        data = np.load(path)
        return data["positions"]
    return None


def objective_sim(trial, all_demos):
    """Optuna 目标函数 — 优化 ProMP 超参数, 最小化 LOO 泛化误差 喵~

    搜索 ProMP 最优超参数 (n_basis, sigma),
    使得 learn_multiple(all_40) 能最好地代表全部40条心形轨迹。
    """
    n_basis = trial.suggest_int("n_basis", 5, 50)
    sigma = trial.suggest_float("sigma", 0.01, 0.15)

    # Leave-One-Out: 对每条轨迹, 用其余39条训练, 测试泛化误差
    from latte_imitation.promp_learner import ProMP3D
    rmses = []
    n = len(all_demos)
    for i in range(n):
        test = all_demos[i]
        train = [all_demos[j] for j in range(n) if j != i]
        p = ProMP3D(n_basis=n_basis, sigma=sigma)
        try:
            p.learn_multiple(train)
            gen = p.generate(T=len(test))
            rmse = np.sqrt(np.mean(np.sum((test-gen)**2, axis=1)))*1000
            rmses.append(rmse)
        except Exception:
            return 0.0

    mean_rmse = np.mean(rmses)
    # 转化为最大化目标: RMSE越小评分越高, 0mm → 100分, 200mm → 0分
    score = max(0.0, 100.0 * (1.0 - mean_rmse / 200.0))
    return score


class OptimizationRunner:
    """贝叶斯优化运行器 — ProMP 条件推理 喵~"""

    def __init__(self, mode="sim", study_name=None, all_demos=None):
        self.mode = mode
        self.all_demos = all_demos or []
        self.study_name = study_name or f"heart_opt_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.study = None
        self.best_params = None
        self.best_score = 0.0
        self.trial_scores = []
        self.trial_params = []
        self.callback = None

    def _objective_wrapper(self, trial):
        score = objective_sim(trial, self.all_demos)
        self.trial_scores.append(score)
        self.trial_params.append(trial.params)
        if score > self.best_score:
            self.best_score = score
            self.best_params = dict(trial.params)
        if self.callback:
            self.callback(trial.number, score, dict(trial.params),
                          self.best_score, self.best_params)
        return score

    def run(self, n_trials=50, timeout=None):
        """运行优化 喵~"""
        print(f"开始贝叶斯优化: {self.study_name}")
        print(f"模式: {self.mode}, 轮数: {n_trials}")
        print(f"采样器: TPE (Tree-structured Parzen Estimator)")

        # 创建 study (支持断点续传)
        storage_path = os.path.join(CONFIG_DIR, f"{self.study_name}.db")
        storage = f"sqlite:///{storage_path}"

        self.study = optuna.create_study(
            study_name=self.study_name,
            storage=storage,
            direction="maximize",
            sampler=optuna.samplers.TPESampler(seed=42),
            load_if_exists=True,
        )

        t0 = time.perf_counter()
        self.study.optimize(
            self._objective_wrapper,
            n_trials=n_trials,
            timeout=timeout,
            show_progress_bar=True,
        )
        elapsed = time.perf_counter() - t0

        self.best_params = self.study.best_params
        self.best_score = self.study.best_value

        print(f"\n{'='*60}")
        print(f"  优化完成!")
        print(f"  耗时: {elapsed:.1f}s ({elapsed/n_trials:.1f}s/trial)")
        print(f"  最优评分: {self.best_score:.1f}")
        print(f"  最优参数:")
        for k, v in self.best_params.items():
            if isinstance(v, float):
                print(f"    {k}: {v:.4f}")
            else:
                print(f"    {k}: {v}")

        # 参数重要性
        try:
            importances = optuna.importance.get_param_importances(self.study)
            print(f"\n  参数重要性:")
            for k, v in sorted(importances.items(), key=lambda x: -x[1]):
                print(f"    {k}: {v:.3f}")
        except Exception:
            pass

        self.save_results()
        return self.best_params, self.best_score

    def save_results(self):
        """保存最优参数为 YAML + JSON 喵~"""
        import yaml

        # YAML
        yaml_path = os.path.join(CONFIG_DIR, "heart_params_optimized.yaml")
        result = {
            "meta": {
                "study_name": self.study_name,
                "mode": self.mode,
                "n_trials": len(self.trial_scores),
                "best_score": round(self.best_score, 2),
                "optimizer": "Optuna TPE",
                "generated_by": "scripts/optimize_heart_params.py",
            },
            "best_proMP_hyperparams": {
                "n_basis": self.best_params.get("n_basis", 20),
                "sigma": self.best_params.get("sigma", 0.03),
            },
        }
        with open(yaml_path, "w") as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        print(f"\n  最优参数已保存: {yaml_path}")

        # JSON (完整 history)
        json_path = yaml_path.replace(".yaml", ".json")
        history = {
            "study_name": self.study_name,
            "mode": self.mode,
            "best_score": self.best_score,
            "best_params": self.best_params,
            "trial_scores": self.trial_scores,
        }
        with open(json_path, "w") as f:
            json.dump(history, f, indent=2)
        print(f"  优化历史已保存: {json_path}")


def main():
    parser = argparse.ArgumentParser(
        description="心形参数贝叶斯优化",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="GUI中直接点击「开始优化」即可, 无需手动运行本脚本",
    )
    parser.add_argument("--real", action="store_true", default=False,
                        help="真机模式 (默认: 模拟)")
    parser.add_argument("--trials", type=int, default=50,
                        help="优化轮数")
    parser.add_argument("--study", type=str, default="heart_opt",
                        help="Study名称, 支持断点续传")
    args = parser.parse_args()

    mode = "real" if args.real else "sim"
    ref = extract_forming_ref()

    runner = OptimizationRunner(mode=mode, study_name=args.study, ref_traj=ref)
    runner.run(n_trials=args.trials)


if __name__ == "__main__":
    main()
