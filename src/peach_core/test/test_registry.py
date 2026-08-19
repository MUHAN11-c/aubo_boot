"""Registry[T]：注册/重名抛错/按名创建/装饰器/未注册报错信息."""
import unittest

from peach_core.registry import Registry


class _Base:
    def __init__(self, gain=1.0):
        self.gain = gain


class _ImplA(_Base):
    pass


class _ImplB(_Base):
    pass


class RegistryTest(unittest.TestCase):
    """2.14 装配规则注册表行为."""

    def test_direct_register_and_create_with_kwargs(self):
        """直接注册后按名创建，kwargs 透传构造函数."""
        reg = Registry('检测器')
        reg.register('a', _ImplA)
        inst = reg.create('a', gain=2.5)
        self.assertIsInstance(inst, _ImplA)
        self.assertEqual(inst.gain, 2.5)

    def test_decorator_register(self):
        """@reg.register('name') 装饰器用法，类对象原样返回."""
        reg = Registry('检测器')

        @reg.register('yolo')
        class Yolo(_Base):
            pass

        self.assertIsInstance(reg.create('yolo'), Yolo)

    def test_duplicate_name_raises_with_name_in_message(self):
        """同名重复注册抛 ValueError，错误信息含名称."""
        reg = Registry('检测器')
        reg.register('a', _ImplA)
        with self.assertRaises(ValueError) as ctx:
            reg.register('a', _ImplB)
        self.assertIn('a', str(ctx.exception))

    def test_duplicate_via_decorator_raises(self):
        """装饰器路径同名同样抛 ValueError."""
        reg = Registry('检测器')
        reg.register('a', _ImplA)
        with self.assertRaises(ValueError):
            reg.register('a')(_ImplB)

    def test_unknown_name_raises_listing_available(self):
        """未注册名抛 KeyError，错误信息含请求名与全部可用名."""
        reg = Registry('检测器')
        reg.register('a', _ImplA)
        reg.register('b', _ImplB)
        with self.assertRaises(KeyError) as ctx:
            reg.create('nope')
        message = str(ctx.exception)
        self.assertIn('nope', message)
        self.assertIn('a', message)
        self.assertIn('b', message)

    def test_names_sorted_and_contains(self):
        """names() 返回排序元组；in 查询注册名."""
        reg = Registry('检测器')
        self.assertEqual(reg.names(), ())
        reg.register('b', _ImplB)
        reg.register('a', _ImplA)
        self.assertEqual(reg.names(), ('a', 'b'))
        self.assertIn('a', reg)
        self.assertNotIn('zzz', reg)


if __name__ == '__main__':
    unittest.main()
