"""自产 XML（SDF/URDF）安全解析：产物不含 DTD/实体，出现即拒绝（防实体扩展）."""

import xml.etree.ElementTree as ET


def parse_xml(text: str) -> ET.Element:
    """解析自产 XML 文本；含 DTD/实体声明则拒绝（billion-laughs 防护）."""
    lowered = text.lower()
    if '<!doctype' in lowered or '<!entity' in lowered:
        raise ValueError('XML 含 DTD/实体声明，拒绝解析')
    return ET.fromstring(text)
