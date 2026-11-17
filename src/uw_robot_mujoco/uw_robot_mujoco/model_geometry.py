"""Read allocation geometry from the supported AUV XML without loading meshes."""
from pathlib import Path
import xml.etree.ElementTree as ET


def _vector(element, name, default):
    values = [float(x) for x in element.get(name, default).split()]
    if len(values) != 3:
        raise ValueError('Expected three coordinates: ' + name)
    return values


def _add(a, b):
    return [x+y for x, y in zip(a, b)]


def _rotation(body):
    # The AUV body tree uses quaternions; reject other orientations rather than guess.
    if any(key in body.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
        raise ValueError('Allocation geometry requires quaternion body orientations')
    q = [float(x) for x in body.get('quat', '1 0 0 0').split()]
    if len(q) != 4:
        raise ValueError('Invalid body quaternion')
    norm = sum(x*x for x in q)**.5
    if norm <= 1e-12:
        raise ValueError('Invalid zero body quaternion')
    w, x, y, z = [v/norm for v in q]
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def _apply(rotation, vector):
    return [sum(a*b for a, b in zip(row, vector)) for row in rotation]


def allocation_geometry_parameters(model_path):
    """读取本 AUV 的 base + 四个单关节组件；启动阶段不重复编译 STL 模型。"""
    roots = []
    visited = set()

    def read(path):
        path = Path(path).resolve()
        if path in visited:
            return
        visited.add(path)
        root = ET.parse(path).getroot()
        roots.append(root)
        for include in root.findall('.//include'):
            read(path.parent / include.attrib['file'])

    read(model_path)
    bases = [body for root in roots for body in root.findall('.//body') if body.get('name') == 'base']
    if len(bases) != 1:
        raise ValueError('Expected one AUV base body')
    base = bases[0]
    inertial = base.find('inertial')
    if inertial is None:
        raise ValueError('AUV base requires explicit inertial data')
    total = float(inertial.attrib['mass'])
    weighted = [total*x for x in _vector(inertial, 'pos', '0 0 0')]
    sites, hinges, axes, centers, masses = [], [], [], [], []
    for number, sign in [('1', 1), ('3', 1), ('2', 1), ('4', -1)]:
        child = next(body for body in base.findall('body') if body.get('name') == number)
        if child.findall('body'):
            raise ValueError('Nested actuator components require an updated geometry exporter')
        joint = child.find('joint')
        site = next(site for site in child.findall('site') if site.get('name') == 'thrust'+number)
        part = child.find('inertial')
        if joint is None or part is None:
            raise ValueError('Missing actuator joint or explicit inertial data')
        if any(key in site.attrib for key in ('quat', 'euler', 'axisangle', 'xyaxes', 'zaxis')):
            raise ValueError('AUV thrust sites must use their body +X direction')
        rotation = _rotation(child)
        origin = _vector(child, 'pos', '0 0 0')
        pivot = _vector(joint, 'pos', '0 0 0')
        center = _vector(part, 'pos', '0 0 0')
        mass = float(part.attrib['mass'])
        if mass <= 0:
            raise ValueError('Actuator mass must be positive')
        total += mass
        weighted = _add(weighted, [mass*x for x in _add(origin, _apply(rotation, center))])
        sites.extend(_add(origin, _apply(rotation, _vector(site, 'pos', '0 0 0'))))
        hinges.extend(_add(origin, _apply(rotation, pivot)))
        axes.extend([sign*x for x in _apply(rotation, _vector(joint, 'axis', '0 0 1'))])
        centers.extend(_apply(rotation, [a-b for a, b in zip(center, pivot)]))
        masses.append(mass)
    if total <= 0 or len(base.findall('body')) != 4:
        raise ValueError('Unexpected AUV body tree')
    return {
        'rotating_thrust_sites': True,
        'geometry_center_of_mass': [x/total for x in weighted],
        'geometry_sites': sites,
        'geometry_hinges': hinges,
        'geometry_axes': axes,
        'geometry_child_centers': centers,
        'geometry_mass_fractions': [x/total for x in masses],
    }


def model_mass_kg(model_path):
    """读取同一 XML 的显式部件质量，用于中性浮力的静力矩前馈。"""
    path = Path(model_path).resolve()
    root = ET.parse(path).getroot()
    base = next((body for body in root.findall('.//body') if body.get('name') == 'base'), None)
    if base is None:
        for include in root.findall('.//include'):
            try:
                return model_mass_kg(path.parent/include.attrib['file'])
            except ValueError:
                continue
        raise ValueError('Missing AUV base inertial data')
    mass = 0.0
    for body in [base, *base.findall('.//body')]:
        inertial = body.find('inertial')
        if inertial is None:
            raise ValueError('Explicit component mass required')
        mass += float(inertial.attrib['mass'])
    if mass <= 0:
        raise ValueError('Invalid AUV mass')
    return mass
