"""
IVG Web 网关入口。

核心流程：
1. HTTP：/auth/token 校验用户后发 JWT；其它 /api/v1/* 做业务与急停占位。
2. WebSocket /ws/ros：浏览器带 ?token= 连接；网关再连本机 rosbridge，双向转发 JSON。
3. 仅「浏览器→rosbridge」方向做限流、角色策略、服务禁区、publish 去抖；可选空闲熔断急停。
"""
from __future__ import annotations

import asyncio
import logging
import time
from contextlib import asynccontextmanager
from typing import Annotated

import uvicorn
import websockets
from fastapi import Depends, FastAPI, HTTPException, Query, Request, WebSocket, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordRequestForm
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded
from slowapi.util import get_remote_address
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession
from starlette.websockets import WebSocketState

from ivg_gateway.auth_core import create_access_token, decode_token, hash_password, verify_password
from ivg_gateway.config import get_settings
from ivg_gateway.database import SessionLocal, engine, init_db
from ivg_gateway.deps import get_current_user, get_db, require_operator
from ivg_gateway.models import Task, TaskStatus, TrajectoryRecord, User, UserRole
from ivg_gateway.schemas import (
    TaskCreate,
    TaskRead,
    Token,
    TrajectoryCreate,
    TrajectoryRead,
    UserRead,
    VisionInferenceRequest,
    VisionInferenceResponse,
)
from ivg_gateway.services.audit import write_audit
from ivg_gateway.services.safety import emergency_stop_sync
from ivg_gateway.services.ws_policy import (
    ClientMessageRateLimiter,
    PublishDebouncer,
    check_outbound_json,
    parse_rosbridge_op,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ivg_gateway")

settings = get_settings()
# slowapi：按客户端 IP 限制 /auth/token 调用频率，防爆破
limiter = Limiter(key_func=get_remote_address)


async def ensure_bootstrap_user() -> None:
    """若 users 表为空，创建首个管理员（用户名/密码来自 IVG_GATEWAY_BOOTSTRAP_* 配置）。"""
    async with SessionLocal() as session:
        n = await session.scalar(select(func.count()).select_from(User))
        if n and n > 0:
            return
        u = settings.bootstrap_admin_username
        p = settings.bootstrap_admin_password
        user = User(
            username=u,
            hashed_password=hash_password(p),
            role=UserRole.admin.value,
        )
        session.add(user)
        await session.commit()
        logger.warning(
            "已创建初始管理员 %s（请在生产环境修改密码并设置 IVG_GATEWAY_BOOTSTRAP_ADMIN_PASSWORD）",
            u,
        )


@asynccontextmanager
async def lifespan(app: FastAPI):
    """进程启动：建表 + 种子用户；退出：释放数据库引擎。"""
    await init_db()
    await ensure_bootstrap_user()
    yield
    await engine.dispose()


app = FastAPI(
    title="IVG Web Gateway",
    description="浏览器经此网关连接 rosbridge；认证、审计、限流与策略占位。",
    version="0.1.0",
    lifespan=lifespan,
)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
async def health():
    """负载均衡 / 存活探测用。"""
    return {"status": "ok", "service": "ivg_gateway"}


@app.post("/auth/token", response_model=Token)
@limiter.limit(f"{settings.auth_rate_limit_per_minute}/minute")
async def login(
    request: Request,
    session: Annotated[AsyncSession, Depends(get_db)],
    form: Annotated[OAuth2PasswordRequestForm, Depends()],
):
    """OAuth2 密码模式：表单字段 username/password；返回 access_token 供 Bearer 或 WS ?token=。"""
    r = await session.execute(select(User).where(User.username == form.username))
    user = r.scalar_one_or_none()
    if user is None or not verify_password(form.password, user.hashed_password):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="用户名或密码错误")
    if not user.is_active:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="账户已禁用")
    await write_audit(session, username=user.username, action="login", detail=None, user_id=user.id)
    token = create_access_token(user.username, user.role)
    return Token(access_token=token)


@app.get("/auth/me", response_model=UserRead)
async def me(user: Annotated[User, Depends(get_current_user)]):
    """校验 Bearer 有效并返回当前用户档案。"""
    return user


@app.post("/api/v1/safety/emergency-stop")
async def api_emergency_stop(
    request: Request,
    user: Annotated[User, Depends(require_operator)],
    session: Annotated[AsyncSession, Depends(get_db)],
):
    """HTTP 触发急停：同步 roslibpy 在后台线程执行，避免阻塞事件循环。"""
    ok, msg = await asyncio.to_thread(emergency_stop_sync, settings)
    await write_audit(
        session,
        username=user.username,
        action="emergency_stop",
        detail=f"ok={ok} {msg}",
        user_id=user.id,
    )
    return {"ok": ok, "message": msg}


@app.get("/api/v1/tasks", response_model=list[TaskRead])
async def list_tasks(
    user: Annotated[User, Depends(get_current_user)],
    session: Annotated[AsyncSession, Depends(get_db)],
    limit: int = 50,
):
    """任务列表占位：后续可接调度/状态机。"""
    r = await session.execute(select(Task).order_by(Task.created_at.desc()).limit(min(limit, 200)))
    return list(r.scalars().all())


@app.post("/api/v1/tasks", response_model=TaskRead)
async def create_task(
    body: TaskCreate,
    user: Annotated[User, Depends(require_operator)],
    session: Annotated[AsyncSession, Depends(get_db)],
):
    """创建任务记录；viewer 不可调用。"""
    t = Task(name=body.name, payload_json=body.payload_json, created_by_id=user.id, status=TaskStatus.pending.value)
    session.add(t)
    await session.commit()
    await session.refresh(t)
    await write_audit(session, username=user.username, action="task_create", detail=t.id, user_id=user.id)
    return t


@app.get("/api/v1/trajectories", response_model=list[TrajectoryRead])
async def list_trajectories(
    user: Annotated[User, Depends(get_current_user)],
    session: Annotated[AsyncSession, Depends(get_db)],
    limit: int = 50,
):
    """轨迹记录列表占位。"""
    r = await session.execute(
        select(TrajectoryRecord).order_by(TrajectoryRecord.created_at.desc()).limit(min(limit, 200))
    )
    return list(r.scalars().all())


@app.post("/api/v1/trajectories", response_model=TrajectoryRead)
async def create_trajectory(
    body: TrajectoryCreate,
    user: Annotated[User, Depends(require_operator)],
    session: Annotated[AsyncSession, Depends(get_db)],
):
    """写入一条轨迹/路径 JSON 占位。"""
    tr = TrajectoryRecord(name=body.name, data_json=body.data_json, task_id=body.task_id)
    session.add(tr)
    await session.commit()
    await session.refresh(tr)
    await write_audit(
        session, username=user.username, action="trajectory_create", detail=tr.id, user_id=user.id
    )
    return tr


@app.post("/api/v1/ai/vision", response_model=VisionInferenceResponse)
async def ai_vision_stub(
    body: VisionInferenceRequest,
    user: Annotated[User, Depends(require_operator)],
):
    """视觉/大模型推理占位：在此接入真实推理服务。"""
    _ = (body, user)
    return VisionInferenceResponse()


async def _rosbridge_uri() -> str:
    """网关作为客户端连接 rosbridge 的 WebSocket URL（与浏览器无关）。"""
    return f"ws://{settings.rosbridge_host}:{settings.rosbridge_port}/"


@app.websocket("/ws/ros")
async def rosbridge_proxy(websocket: WebSocket, token: str | None = Query(None)):
    """
    浏览器 roslib 连接点：与直连 rosbridge 协议相同，仅多 Query token。
    rb：上游 rosbridge WebSocket；client_to_rb / rb_to_client 为双工泵。
    """
    if not token:
        await websocket.close(code=4401, reason="missing token")
        return
    payload = decode_token(token)
    if not payload or "sub" not in payload:
        await websocket.close(code=4401, reason="invalid token")
        return
    username = str(payload["sub"])  # JWT 主题：登录名
    role = str(payload.get("role") or UserRole.operator.value)  # 用于 WS 出站策略（viewer 限权）

    async with SessionLocal() as session:
        r = await session.execute(select(User).where(User.username == username))
        user = r.scalar_one_or_none()
        if user is None or not user.is_active:
            await websocket.close(code=4401, reason="user inactive")
            return

    await websocket.accept()
    async with SessionLocal() as session:
        await write_audit(session, username=username, action="ws_ros_connect", detail=None, user_id=user.id)

    rate = ClientMessageRateLimiter(settings.ws_client_messages_per_second)  # 每连接每秒上行条数上限
    debouncer = PublishDebouncer(settings.publish_debounce_ms)  # 按 topic 合并过密的 publish
    deny = settings.deny_service_list  # call_service 禁区名子串列表
    last_activity = [time.monotonic()]  # 列表可变：供 client_to_rb 与 idle_watch 共享「上次上行时间」
    stop_idle = asyncio.Event()  # 结束代理时置位，让空闲监控协程退出

    async def idle_watch():
        if settings.idle_stop_seconds <= 0:
            return
        try:
            while not stop_idle.is_set():
                await asyncio.sleep(1.0)
                if stop_idle.is_set():
                    break
                idle = time.monotonic() - last_activity[0]
                if idle < settings.idle_stop_seconds:
                    continue
                ok, msg = await asyncio.to_thread(emergency_stop_sync, settings)
                last_activity[0] = time.monotonic()
                async with SessionLocal() as s:
                    await write_audit(
                        s,
                        username=username,
                        action="circuit_idle_stop",
                        detail=f"idle={idle:.1f}s ok={ok} {msg}",
                        user_id=user.id,
                    )
                logger.warning("idle circuit: user=%s ok=%s %s", username, ok, msg)
        except asyncio.CancelledError:
            pass

    idle_task = asyncio.create_task(idle_watch())

    try:
        async with websockets.connect(
            await _rosbridge_uri(),
            max_size=None,  # 允许大消息（点云等 JSON）
            ping_interval=20,
            ping_timeout=60,
        ) as rb:

            async def client_to_rb():
                """浏览器 → rosbridge：过限流与策略后再 rb.send。"""
                while True:
                    msg = await websocket.receive()
                    if msg["type"] == "websocket.disconnect":
                        break
                    if msg["type"] != "websocket.receive":
                        continue
                    text = msg.get("text")
                    data = text if text is not None else msg.get("bytes")
                    if data is None:
                        continue
                    if isinstance(data, bytes):
                        data = data.decode("utf-8", errors="replace")
                    ok_r, reason_r = rate.allow()
                    if not ok_r:
                        logger.info("ws rate drop: %s %s", username, reason_r)
                        continue
                    ok_j, reason_j = check_outbound_json(role, data, deny_substrings=deny)
                    if not ok_j:
                        logger.info("ws policy drop: %s %s", username, reason_j)
                        async with SessionLocal() as s:
                            await write_audit(
                                s,
                                username=username,
                                action="ws_policy_reject",
                                detail=reason_j[:512],
                                user_id=user.id,
                            )
                        continue
                    op, obj = parse_rosbridge_op(data)
                    if op == "publish" and isinstance(obj, dict):
                        topic = obj.get("topic")
                        if isinstance(topic, str):
                            ok_d, _ = debouncer.should_send_publish(topic)
                            if not ok_d:
                                continue
                    last_activity[0] = time.monotonic()  # 有上行则刷新空闲计时
                    await rb.send(data)

            async def rb_to_client():
                """rosbridge → 浏览器：透明转发。"""
                while True:
                    raw = await rb.recv()
                    if isinstance(raw, bytes):
                        await websocket.send_bytes(raw)
                    else:
                        await websocket.send_text(raw)

            # 任一侧异常或断开则结束整段代理，并取消另一侧任务
            done, pending = await asyncio.wait(
                [asyncio.create_task(client_to_rb()), asyncio.create_task(rb_to_client())],
                return_when=asyncio.FIRST_COMPLETED,
            )
            for p in pending:
                p.cancel()
                try:
                    await p
                except asyncio.CancelledError:
                    pass
            for d in done:
                if d.cancelled():
                    continue
                exc = d.exception()
                if exc:
                    logger.debug("proxy task end: %s", exc)
    except websockets.exceptions.WebSocketException as e:
        logger.info("rosbridge upstream: %s", e)
    except Exception as e:
        logger.exception("rosbridge proxy: %s", e)
    finally:
        stop_idle.set()
        idle_task.cancel()
        try:
            await idle_task
        except asyncio.CancelledError:
            pass
        if websocket.application_state == WebSocketState.CONNECTED:
            try:
                await websocket.close()
            except Exception:
                pass
        async with SessionLocal() as session:
            await write_audit(session, username=username, action="ws_ros_disconnect", detail=None, user_id=user.id)


def main():
    uvicorn.run(
        "ivg_gateway.main:app",
        host=settings.host,
        port=settings.port,
        reload=False,
    )


if __name__ == "__main__":
    main()
