"""
实现注册表 Registry[T] — 2.14 装配规则的纯核底座.

职责:
  按名注册/创建可替换实现（检测器、分割器、拟合器等），支撑「配置选
  实现」的装配规则：编排层只依赖接口基类 + 注册表，具体实现以
  @REGISTRY.register('name') 自登记。

输入/输出契约:
  register(name, cls) 直接注册或作装饰器；同名重复注册抛 ValueError
  （错误信息含名称）；create(name, **kwargs) 按名实例化，未注册抛
  KeyError（错误信息列出全部可用名称）；names() 返回已注册名称元组。

协议条款:
  2.14 装配规则；纯核零 ROS import，零第三方依赖。

线程模型:
  注册发生在模块导入期（单线程），运行期只读；无内部锁——若运行期
  动态注册需调用方自行同步。
"""
from __future__ import annotations

from typing import Dict, Generic, Type, TypeVar


T = TypeVar('T')


class Registry(Generic[T]):
    """
    按名注册/创建可替换实现的注册表（2.14 装配规则）.

    用法::

        DETECTORS: Registry[Detector] = Registry('检测器')

        @DETECTORS.register('yolo')
        class YoloDetector(Detector):
            ...

        detector = DETECTORS.create('yolo', weights='best.pt')

    生命周期：模块级单例，随进程存活。线程安全：导入期注册、运行期
    只读（见模块 docstring）。
    """

    def __init__(self, kind: str = '组件'):
        """创建空注册表；kind 为错误信息中的类别名（如 '检测器'）."""
        self._kind = kind
        self._items: Dict[str, Type[T]] = {}

    def register(self, name: str, cls: Type[T] = None):
        """
        注册实现类；支持直接调用与装饰器两种写法.

        Args:
            name: 注册名（配置文件中选择实现的键）.
            cls: 实现类；省略时返回装饰器.

        Raises
        ------
            ValueError: 同名重复注册（错误信息含名称）.

        """
        def _do_register(impl: Type[T]) -> Type[T]:
            if name in self._items:
                raise ValueError(
                    f'{self._kind}名称重复注册: {name!r}')
            self._items[name] = impl
            return impl

        if cls is None:
            return _do_register
        return _do_register(cls)

    def create(self, name: str, **kwargs) -> T:
        """
        按名实例化已注册实现，kwargs 透传构造函数.

        Raises
        ------
            KeyError: 名称未注册（错误信息列出全部可用名称）.

        """
        try:
            cls = self._items[name]
        except KeyError:
            available = ', '.join(self.names()) or '（空）'
            raise KeyError(
                f'未注册的{self._kind}: {name!r}，可用: {available}') from None
        return cls(**kwargs)

    def names(self) -> tuple:
        """返回全部已注册名称（排序后的元组，只读）."""
        return tuple(sorted(self._items))

    def __contains__(self, name: str) -> bool:
        """支持 ``name in registry`` 查询."""
        return name in self._items
