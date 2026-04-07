"""
网关运行时配置：环境变量前缀 IVG_GATEWAY_（例 IVG_GATEWAY_SECRET_KEY）。
字段名转大写加前缀即为环境变量名；见各字段说明。
"""
import json
from functools import lru_cache
from typing import List

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_prefix="IVG_GATEWAY_",
        env_file=".env",
        extra="ignore",
    )

    # ---------- HTTP / JWT ----------
    host: str = "0.0.0.0"  # uvicorn 监听地址
    port: int = 8765  # 网关 HTTP 与 WebSocket 同端口
    secret_key: str = Field(
        default="change-me-use-IVG_GATEWAY_SECRET_KEY",
        description="JWT 签名密钥，生产环境必须覆盖",
    )
    access_token_expire_minutes: int = 480  # JWT 有效期（分钟）
    algorithm: str = "HS256"  # JWT 签名算法

    # ---------- 数据库 ----------
    database_url: str = "sqlite+aiosqlite:///./ivg_gateway.db"  # 相对路径相对进程工作目录

    # ---------- 上游 rosbridge（仅服务端连接，浏览器永直连此处）----------
    rosbridge_host: str = "127.0.0.1"
    rosbridge_port: int = 9090  # launch 可通过 IVG_GATEWAY_ROSBRIDGE_PORT 覆盖

    # ---------- 种子管理员（仅 users 表为空时）----------
    bootstrap_admin_username: str = "admin"
    bootstrap_admin_password: str = "changeme"

    # ---------- 安全与策略 ----------
    auth_rate_limit_per_minute: int = 20  # 登录接口每分钟每 IP 次数上限（slowapi）
    ws_client_messages_per_second: int = 200  # 每条浏览器 WS 连接每秒上行条数；0=不限
    publish_debounce_ms: int = 0  # 同一连接上 publish 按 topic 去抖毫秒数；0=关闭
    deny_service_substrings: str = ""  # call_service 禁止的服务名子串，逗号分隔或 JSON 数组
    idle_stop_seconds: float = 0.0  # 上行空闲超此时长触发急停；0=关闭
    emergency_cmd_vel_topic: str = ""  # 急停时发布的 Twist 话题名；空则只打日志
    emergency_twist_message_json: str = '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'

    cors_origins: str = "*"  # 浏览器跨域；生产建议改为具体源列表逗号分隔

    @property
    def deny_service_list(self) -> List[str]:
        raw = (self.deny_service_substrings or "").strip()
        if not raw:
            return []
        if raw.startswith("["):
            try:
                return [str(x).strip() for x in json.loads(raw) if str(x).strip()]
            except json.JSONDecodeError:
                return []
        return [p.strip() for p in raw.split(",") if p.strip()]

    @property
    def cors_origins_list(self) -> List[str]:
        if self.cors_origins.strip() == "*":
            return ["*"]
        return [o.strip() for o in self.cors_origins.split(",") if o.strip()]


@lru_cache
def get_settings() -> Settings:
    """单例配置；进程内缓存，测试时需清缓存或重启进程。"""
    return Settings()
