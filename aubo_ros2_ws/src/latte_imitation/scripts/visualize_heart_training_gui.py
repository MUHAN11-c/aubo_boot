#!/usr/bin/env python3
"""心形拉花训练可视化 — 模块化 GUI 入口 喵~"""

import sys, os, argparse
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow


def main():
    parser = argparse.ArgumentParser(description="心形训练可视化GUI")
    parser.add_argument("--episode", type=int, default=32)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui = MainWindow(episode=args.episode)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
