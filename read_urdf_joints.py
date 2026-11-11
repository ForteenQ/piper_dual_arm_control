import xml.etree.ElementTree as ET

tree = ET.parse('/home/lab203/Desktop/piper_ros-humble/src/piper_description/urdf/piper_description.urdf')
for j in tree.findall('.//joint'):
    print(f"joint: {j.attrib.get('name')}, type: {j.attrib.get('type')}")
