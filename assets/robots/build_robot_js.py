'''
Packs a MuJoCo Menagerie robot into <robot>/<robot>.js, a classic script that index.html loads with <script src>
(unlike fetch(), this also works when the page is opened from disk, file://).
	window.ROBOT_FILES['<robot>/robot.xml'], ['<robot>/assets/...'] : the physics model for MuJoCo, without the visual
		geoms (MuJoCo only needs the collision geometry, which keeps compiling a scene fast)
	window.ROBOT_VISUALS['<robot>'] : the visual geoms { body, mesh, rgba } and their meshes at full resolution
		(vertices and triangles of the OBJ files as they are, float32 / uint16 or uint32 in base64), rendered by three.js
The models :
	franka_emika_panda : panda.xml
	franka_fr3         : fr3.xml with the Franka (Panda) hand of franka_emika_panda attached to fr3_link7
	                     (hand body, finger slide joints, tendon, equality, gripper actuator, hand meshes),
	                     as panda.xml mounts it on panda_link7
Run from the repository root after changing the robot files:
	python assets/robots/build_robot_js.py [robot ...]   (default : all robots)
'''
import os
import re
import sys
import json
import base64
import struct
import xml.etree.ElementTree as ET

ROBOTS_DIR = os.path.dirname(os.path.abspath(__file__))

# Franka hand of panda.xml (franka_emika_panda), for fr3.xml
HAND_DEFAULTS = '''
    <default class="panda">
      <joint armature="0.1" damping="1" axis="0 0 1" range="-2.8973 2.8973"/>
      <general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>
      <default class="finger">
        <joint axis="0 1 0" type="slide" range="0 0.04"/>
      </default>
      <default class="hand_collision">
        <geom type="mesh" group="3"/>
        <default class="fingertip_pad_collision_1">
          <geom type="box" size="0.0085 0.004 0.0085" pos="0 0.0055 0.0445"/>
        </default>
        <default class="fingertip_pad_collision_2">
          <geom type="box" size="0.003 0.002 0.003" pos="0.0055 0.002 0.05"/>
        </default>
        <default class="fingertip_pad_collision_3">
          <geom type="box" size="0.003 0.002 0.003" pos="-0.0055 0.002 0.05"/>
        </default>
        <default class="fingertip_pad_collision_4">
          <geom type="box" size="0.003 0.002 0.0035" pos="0.0055 0.002 0.0395"/>
        </default>
        <default class="fingertip_pad_collision_5">
          <geom type="box" size="0.003 0.002 0.0035" pos="-0.0055 0.002 0.0395"/>
        </default>
      </default>
    </default>
'''
HAND_ASSETS = '''
    <material name="off_white" rgba="0.901961 0.921569 0.929412 1"/>
    <mesh name="hand_c" file="hand.stl"/>
    <mesh file="hand_0.obj"/>
    <mesh file="hand_1.obj"/>
    <mesh file="hand_2.obj"/>
    <mesh file="hand_3.obj"/>
    <mesh file="hand_4.obj"/>
    <mesh file="finger_0.obj"/>
    <mesh file="finger_1.obj"/>
'''
FINGER_GEOMS = '''
                            <geom mesh="finger_0" material="off_white" class="visual"/>
                            <geom mesh="finger_1" material="black" class="visual"/>
                            <geom mesh="finger_0" class="hand_collision"/>
                            <geom class="fingertip_pad_collision_1"/>
                            <geom class="fingertip_pad_collision_2"/>
                            <geom class="fingertip_pad_collision_3"/>
                            <geom class="fingertip_pad_collision_4"/>
                            <geom class="fingertip_pad_collision_5"/>'''
HAND_BODY = f'''
                      <body name="hand" childclass="panda" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">
                        <inertial mass="0.73" pos="-0.01 0 0.03" diaginertia="0.001 0.0025 0.0017"/>
                        <geom mesh="hand_0" material="off_white" class="visual"/>
                        <geom mesh="hand_1" material="black" class="visual"/>
                        <geom mesh="hand_2" material="black" class="visual"/>
                        <geom mesh="hand_3" material="white" class="visual"/>
                        <geom mesh="hand_4" material="off_white" class="visual"/>
                        <geom mesh="hand_c" class="hand_collision"/>
                        <body name="left_finger" pos="0 0 0.0584">
                          <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                          <joint name="finger_joint1" class="finger"/>{FINGER_GEOMS}
                        </body>
                        <body name="right_finger" pos="0 0 0.0584" quat="0 0 0 1">
                          <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                          <joint name="finger_joint2" class="finger"/>{FINGER_GEOMS}
                        </body>
                      </body>'''
HAND_CONSTRAINTS = '''
  <tendon>
    <fixed name="split">
      <joint joint="finger_joint1" coef="0.5"/>
      <joint joint="finger_joint2" coef="0.5"/>
    </fixed>
  </tendon>

  <equality>
    <joint joint1="finger_joint1" joint2="finger_joint2" solimp="0.95 0.99 0.001" solref="0.005 1"/>
  </equality>
'''
HAND_ACTUATOR = '''    <!-- Remap original ctrlrange (0, 0.04) to (0, 255): 0.04 * 100 / 255 = 0.01568627451 -->
    <general class="panda" name="actuator8" tendon="split" forcerange="-100 100" ctrlrange="0 255"
      gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/>
'''

def insert(text, anchor, addition, after=True):
	'''
	output: text with addition inserted after (or before) the single occurrence of anchor
	'''
	assert text.count(anchor) == 1, f'anchor not found exactly once: {anchor}'
	i = text.index(anchor) + (len(anchor) if after else 0)
	return text[:i] + addition + text[i:]

def fr3_with_hand(xml):
	'''
	output: fr3.xml with the Franka hand of panda.xml attached to fr3_link7 (at attachment_site)
	'''
	xml = insert(xml, '\n  </default>\n', HAND_DEFAULTS, after=False)
	xml = insert(xml, '  </asset>', HAND_ASSETS, after=False)
	xml = insert(xml, '<site name="attachment_site" pos="0 0 0.107"/>', HAND_BODY)
	xml = insert(xml, '  <actuator>', HAND_CONSTRAINTS + '\n', after=False)
	xml = insert(xml, '  </actuator>', HAND_ACTUATOR, after=False)
	xml = xml.replace(
		'<key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"/>',
		'<key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853 0.04 0.04" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255"/>')
	return xml

# robot : (root MJCF file, transform of its text, extra mesh folders : (folder, files))
ROBOTS = {
	'franka_emika_panda': ('panda.xml', lambda xml: xml, []),
	'franka_fr3': ('fr3.xml', fr3_with_hand, [('franka_emika_panda/assets', [
		'hand.stl', 'hand_0.obj', 'hand_1.obj', 'hand_2.obj', 'hand_3.obj', 'hand_4.obj', 'finger_0.obj', 'finger_1.obj'])]),
}


def read_obj(path):
	'''
	output: vertices (flat list of floats), triangles (flat list of 0-based indices) of an OBJ file
		(polygons triangulated as fans ; texture coordinates and normals ignored)
	'''
	vertices, triangles = [], []
	with open(path) as f:
		for line in f:
			if line.startswith('v '):
				vertices.extend(float(x) for x in line.split()[1:4])
			elif line.startswith('f '):
				n = len(vertices) // 3
				face = [int(tok.split('/')[0]) for tok in line.split()[1:]]
				face = [i - 1 if i > 0 else n + i for i in face]
				for k in range(1, len(face) - 1):
					triangles.extend([face[0], face[k], face[k + 1]])
	return vertices, triangles

def visual_geoms(xml):
	'''
	output: [{ body, mesh, material }] of the visual geoms (class "visual") of an MJCF text
	'''
	geoms = []
	def visit(element, body):
		for child in element:
			if child.tag == 'body':
				visit(child, child.get('name'))
			elif child.tag == 'geom' and child.get('class') == 'visual':
				assert child.get('pos') is None and child.get('quat') is None, 'visual geoms are expected in the body frame'
				geoms.append({'body': body, 'mesh': child.get('mesh'), 'material': child.get('material')})
	visit(ET.fromstring(xml).find('worldbody'), 'world')
	return geoms

def physics_xml(xml):
	'''
	output: the MJCF text without its visual geoms and the mesh assets no remaining geom uses
	'''
	xml = re.sub(r'\n\s*<geom [^>]*class="visual"[^>]*/>', '', xml)
	used = set(re.findall(r'<geom [^>]*mesh="([^"]+)"', xml))
	def keep(match):
		name = re.search(r'name="([^"]+)"', match.group(0))
		name = name.group(1) if name else os.path.splitext(re.search(r'file="([^"]+)"', match.group(0)).group(1))[0]
		return match.group(0) if name in used else ''
	return re.sub(r'\n\s*<mesh [^>]*/>', keep, xml)

def build(name):
	root, transform, extra = ROBOTS[name]
	robot_dir = os.path.join(ROBOTS_DIR, name)
	xml = transform(open(os.path.join(robot_dir, root)).read())
	sources = {f: os.path.join(robot_dir, 'assets', f) for f in re.findall(r'file="([^"]+)"', xml)}
	for folder, names in extra:
		sources.update({f: os.path.join(ROBOTS_DIR, folder, f) for f in names})
	mesh_files = {}
	for m in re.finditer(r'<mesh [^>]*/>', xml):
		file = re.search(r'file="([^"]+)"', m.group(0)).group(1)
		mesh_name = re.search(r'name="([^"]+)"', m.group(0))
		mesh_files[mesh_name.group(1) if mesh_name else os.path.splitext(file)[0]] = file

	# physics model and its mesh files
	physics = physics_xml(xml)
	entries = [(f'{name}/robot.xml', 'text', physics)]
	for file in re.findall(r'file="([^"]+)"', physics):
		data = open(sources[file], 'rb').read()
		if file.endswith('.obj'):
			entries.append((f'{name}/assets/{file}', 'text', data.decode()))
		else:
			entries.append((f'{name}/assets/{file}', 'base64', base64.b64encode(data).decode()))

	# visual geoms and meshes (full resolution)
	rgba = {m.get('name'): [float(v) for v in m.get('rgba').split()] for m in ET.fromstring(xml).iter('material') if m.get('rgba')}
	geoms = visual_geoms(xml)
	meshes, n_triangles = {}, 0
	for mesh in sorted({g['mesh'] for g in geoms}):
		vertices, triangles = read_obj(sources[mesh_files[mesh]])
		index = 'uint16' if len(vertices) // 3 < 65536 else 'uint32'
		data = struct.pack(f'<{len(vertices)}f', *vertices) + struct.pack(f'<{len(triangles)}{"H" if index == "uint16" else "I"}', *triangles)
		meshes[mesh] = {'vertices': len(vertices) // 3, 'triangles': len(triangles) // 3, 'index': index,
			'data': base64.b64encode(data).decode()}
		n_triangles += len(triangles) // 3
	visuals = {'geoms': [{'body': g['body'], 'mesh': g['mesh'], 'rgba': rgba.get(g['material'], [0.8, 0.8, 0.8, 1])} for g in geoms],
		'meshes': meshes}

	out = os.path.join(robot_dir, f'{name}.js')
	with open(out, 'w') as f:
		f.write(f'// Generated by assets/robots/build_robot_js.py from {root} and its meshes; do not edit.\n')
		f.write('window.ROBOT_FILES = window.ROBOT_FILES || {};\n')
		f.write('window.ROBOT_VISUALS = window.ROBOT_VISUALS || {};\n')
		for key, kind, data in entries:
			if kind == 'text':
				assert '`' not in data and '${' not in data
				f.write(f"window.ROBOT_FILES['{key}'] = `{data}`;\n")
			else:
				f.write(f"window.ROBOT_FILES['{key}'] = 'base64:{data}';\n")
		f.write(f"window.ROBOT_VISUALS['{name}'] = {json.dumps(visuals)};\n")
	print(f'{out}: {len(entries) - 1} physics mesh files, {len(geoms)} visual geoms, {len(meshes)} visual meshes '
		f'({n_triangles} triangles), {os.path.getsize(out) / 1e6:.1f} MB')

if __name__ == '__main__':
	for name in sys.argv[1:] or ROBOTS:
		build(name)
