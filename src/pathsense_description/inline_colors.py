#!/usr/bin/env python3
"""Write each visual's colour directly into it (RViz then always shows the right colour).
Run once from the package folder:  python3 inline_colors.py"""
import re, os
U = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf')
mats = dict(re.findall(r'<material name="(\w+)">\s*<color rgba="([^"]+)"/>', open(os.path.join(U, 'materials.xacro')).read()))
for f in ('pathsense.urdf.xacro', 'macros.xacro'):
    p = os.path.join(U, f); s = open(p).read()
    s2, n = re.subn(r'<material name="(\w+)"/>',
                    lambda m: f'<material name="{m.group(1)}"><color rgba="{mats[m.group(1)]}"/></material>', s)
    open(p, 'w').write(s2); print(f'{f}: {n} visuals updated')