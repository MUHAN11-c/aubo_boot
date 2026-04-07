"""SQLAlchemy ORM：用户、审计、任务与轨迹占位表。"""
import enum
import uuid
from datetime import datetime, timezone

from sqlalchemy import Boolean, DateTime, ForeignKey, String, Text
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship


class Base(DeclarativeBase):
    """元数据聚合根，供 database.init_db create_all 使用。"""
    pass


class UserRole(str, enum.Enum):
    """viewer：只读型 WS 策略；operator/admin：可 publish/call_service 等。"""

    viewer = "viewer"
    operator = "operator"
    admin = "admin"


class User(Base):
    __tablename__ = "users"

    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    username: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    hashed_password: Mapped[str] = mapped_column(String(255))  # bcrypt，非明文
    role: Mapped[str] = mapped_column(String(32), default=UserRole.operator.value)
    is_active: Mapped[bool] = mapped_column(Boolean, default=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=lambda: datetime.now(timezone.utc)
    )

    audit_logs: Mapped[list["AuditLog"]] = relationship(back_populates="user", lazy="selectin")
    tasks: Mapped[list["Task"]] = relationship(back_populates="creator", lazy="selectin")


class AuditLog(Base):
    """操作审计：登录、WS 连接、策略拒绝、急停等。"""

    __tablename__ = "audit_logs"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=lambda: datetime.now(timezone.utc), index=True
    )
    username: Mapped[str] = mapped_column(String(64), index=True)
    action: Mapped[str] = mapped_column(String(128), index=True)  # 如 login、ws_ros_connect
    detail: Mapped[str | None] = mapped_column(Text, nullable=True)
    user_id: Mapped[str | None] = mapped_column(String(36), ForeignKey("users.id"), nullable=True)

    user: Mapped["User | None"] = relationship(back_populates="audit_logs")


class TaskStatus(str, enum.Enum):
    """任务状态机占位枚举。"""

    pending = "pending"
    running = "running"
    done = "done"
    failed = "failed"


class Task(Base):
    """业务任务占位：名称 + 任意 JSON 字符串 payload。"""

    __tablename__ = "tasks"

    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    name: Mapped[str] = mapped_column(String(256))
    status: Mapped[str] = mapped_column(String(32), default=TaskStatus.pending.value)
    payload_json: Mapped[str | None] = mapped_column(Text, nullable=True)  # 调用方自定义结构
    created_by_id: Mapped[str | None] = mapped_column(String(36), ForeignKey("users.id"), nullable=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=lambda: datetime.now(timezone.utc)
    )

    creator: Mapped["User | None"] = relationship(back_populates="tasks")
    trajectories: Mapped[list["TrajectoryRecord"]] = relationship(back_populates="task", lazy="selectin")


class TrajectoryRecord(Base):
    """轨迹/路径记录占位：可关联 task_id。"""

    __tablename__ = "trajectory_records"

    id: Mapped[str] = mapped_column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    name: Mapped[str] = mapped_column(String(256))
    data_json: Mapped[str | None] = mapped_column(Text, nullable=True)  # 路径点序列等 JSON
    task_id: Mapped[str | None] = mapped_column(String(36), ForeignKey("tasks.id"), nullable=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=lambda: datetime.now(timezone.utc)
    )

    task: Mapped["Task | None"] = relationship(back_populates="trajectories")
