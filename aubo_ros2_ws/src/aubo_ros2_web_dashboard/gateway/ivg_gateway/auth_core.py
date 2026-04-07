"""密码哈希（bcrypt）与 JWT 签发/校验；载荷字段 sub=用户名、role=角色。"""
from datetime import datetime, timedelta, timezone

from jose import JWTError, jwt
from passlib.context import CryptContext

from ivg_gateway.config import get_settings
from ivg_gateway.models import UserRole

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
settings = get_settings()


def verify_password(plain: str, hashed: str) -> bool:
    return pwd_context.verify(plain, hashed)


def hash_password(plain: str) -> str:
    return pwd_context.hash(plain)


def create_access_token(username: str, role: str) -> str:
    """生成带过期时间的 HS256 JWT；role 会写入 WS 策略分支。"""
    expire = datetime.now(timezone.utc) + timedelta(minutes=settings.access_token_expire_minutes)
    payload = {"sub": username, "role": role, "exp": expire}
    return jwt.encode(payload, settings.secret_key, algorithm=settings.algorithm)


def decode_token(token: str) -> dict | None:
    """校验签名与 exp；失败返回 None。"""
    try:
        return jwt.decode(token, settings.secret_key, algorithms=[settings.algorithm])
    except JWTError:
        return None


def role_may_control_ros(role: str) -> bool:
    """是否具备「控制类」REST 权限（与 WS 里 viewer 拦截互补，可供后续扩展）。"""
    return role in (UserRole.operator.value, UserRole.admin.value)
