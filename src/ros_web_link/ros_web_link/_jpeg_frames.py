# ros_web_link/_jpeg_frames.py — 真实可解码棋盘 JPEG 帧（PIL 预生成，base64 内嵌）
#
# 两相位交替的 64×48 mono 棋盘（与 /rwl_viz/image 同图案）：web 端 CompressedImage
# 显示链路的真解码验证数据。由 PIL(q=80) 生成后 base64 内嵌——fixture 零新增运行时
# 依赖；再生：见 viz_fixture 模块 docstring。

import base64

_FRAME_0 = (
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGB'
    'cUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/wAALCAAwAEABAREA/8QAHwAAAQUB'
    'AQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMU'
    'EGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpT'
    'VFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7'
    'i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/APqmvyrr'
    '9VK/Kuv1Ur8q6/VSvyror9VK/Kuv1Ur8q6/VSvyrr9VKK/Kuv1Ur8q6/VSvyrr9VK/Kuiv1Ur8'
    'q6/VSvyrr9VK/Kuv1Uor8q6/VSvyrr9VK/Kuv1Ur8q6K/VSvyrr9VK/Kuv1Ur8q6/VSv/Z'
)

_FRAME_1 = (
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGB'
    'cUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/wAALCAAwAEABAREA/8QAHwAAAQUB'
    'AQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMU'
    'EGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpT'
    'VFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7'
    'i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/APlSv1Ur'
    '8q6/VSvyrr9VK/Kuv1Uor8q6/VSvyrr9VK/Kuv1Ur8q6K/VSvyrr9VK/Kuv1Ur8q6/VSipyrr9'
    'VK/Kuv1Ur8q6/VSvyror9VK/Kuv1Ur8q6/VSvyrr9VKK/Kuv1Ur8q6/VSvyrr9VK/Kuv/Z'
)

JPEG_FRAMES = (
    base64.b64decode("".join(_FRAME_0)),
    base64.b64decode("".join(_FRAME_1)),
)
