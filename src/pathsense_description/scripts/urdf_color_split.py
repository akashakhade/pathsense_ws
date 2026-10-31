#!/usr/bin/env python3
"""Run xacro, then give every colour its own (fixed) child link.

RViz paints all visuals of a link with the colour of that link's first visual,
so a link with several colours (silver wheel faces on a black track ring, orange
arm beam on a black belt, ...) shows up dark. Splitting the visuals by colour into
fixed child links makes RViz show every colour correctly. Gazebo merges fixed
links back together, so physics and sensors are unchanged.

Usage (prints the final URDF):  urdf_color_split.py <file.urdf.xacro> [xacro_arg:=value ...]
"""
import subprocess
import sys
import xml.etree.ElementTree as ET


def material_key(visual):
    m = visual.find('material')
    if m is None:
        return None
    c = m.find('color')
    return m.get('name') or (c.get('rgba') if c is not None else None)


def split(urdf_xml):
    robot = ET.fromstring(urdf_xml)
    new_links, new_joints = [], []
    for link in robot.findall('link'):
        visuals = link.findall('visual')
        keys = []
        for v in visuals:
            k = material_key(v)
            if k is not None and k not in keys:
                keys.append(k)
        if len(keys) < 2:
            continue
        first = material_key(visuals[0]) or keys[0]
        for k in keys:
            if k == first:
                continue
            safe = ''.join(ch if ch.isalnum() else '_' for ch in k)
            child = ET.Element('link', {'name': f"{link.get('name')}_{safe}"})
            for v in visuals:
                if material_key(v) == k:
                    link.remove(v)
                    child.append(v)
            joint = ET.Element('joint', {'name': f"{link.get('name')}_{safe}_joint", 'type': 'fixed'})
            ET.SubElement(joint, 'parent', {'link': link.get('name')})
            ET.SubElement(joint, 'child', {'link': child.get('name')})
            ET.SubElement(joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            new_links.append(child)
            new_joints.append(joint)
    for e in new_links + new_joints:
        robot.append(e)
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode')


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    if src.endswith('.xacro'):
        xml = subprocess.run(['xacro', src] + sys.argv[2:], check=True,
                             capture_output=True, text=True).stdout
    else:
        xml = open(src).read()
    sys.stdout.write(split(xml))


if __name__ == '__main__':
    main()
