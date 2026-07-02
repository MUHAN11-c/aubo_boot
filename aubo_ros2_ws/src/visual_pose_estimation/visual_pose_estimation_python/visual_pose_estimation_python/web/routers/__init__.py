"""按领域划分的 APIRouter 子模块（均在 app.create_app 中 include_router）。"""

from . import camera, debug, grasp, pose, robot, system, templates

__all__ = ["camera", "debug", "grasp", "pose", "robot", "system", "templates"]
