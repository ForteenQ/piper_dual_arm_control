import xml.etree.ElementTree as ET

tree = ET.parse('/home/lab203/Desktop/piper_ros-humble/src/piper_description/urdf/piper_description.urdf')
for j in tree.findall('.//joint'):
    name = j.attrib.get('name')
    limit = j.find('limit')
    if limit is not None:
        lower = limit.attrib.get('lower')
        upper = limit.attrib.get('upper')
        effort = limit.attrib.get('effort')
        velocity = limit.attrib.get('velocity')
        print(f"{name:10s} | lower: {lower:8s} | upper: {upper:8s} | vel: {velocity:5s}")
