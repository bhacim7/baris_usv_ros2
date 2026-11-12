"""
The generated world (scripts/make_course_world.py): one boat model per boat (step 6e).

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_world.py -v
"""

import importlib.util
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'make_course_world', os.path.join(HERE, '..', 'scripts', 'make_course_world.py'))
mcw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mcw)


def topics(model):
    return sorted({e.text for e in model.iter()
                   if e.tag in ('topic', 'odom_topic') and e.text and 'usv' in e.text})


def test_each_boat_has_its_own_name_and_topics():
    a = mcw.boat_model('azure', 1.0, 2.0, 0.5)
    m = mcw.boat_model('marine', 1.0, 8.0, 0.5)
    assert (a.get('name'), m.get('name')) == ('usv_azure', 'usv_marine')
    assert all(t.startswith('/usv_azure/') for t in topics(a)) and topics(a)
    assert all(t.startswith('/usv_marine/') for t in topics(m))
    assert {e.text for e in m.iter('namespace')} == {'usv_marine'}
    # One hull mesh for all
    assert {e.text for e in m.iter('uri')} == {'file://usv_boat/meshes/hull.obj'}
    assert a.find('pose').text == '1.000 2.000 0 0 0 0.5000'


def test_world_for_two_boats(tmp_path):
    out = tmp_path / 'w.sdf'
    course = os.path.join(HERE, '..', 'courses', 'roboboat_2027.yaml')
    argv = sys.argv
    sys.argv = ['make_course_world.py', course, str(out), 'azure,marine']
    try:
        mcw.main()
    finally:
        sys.argv = argv
    world = ET.parse(out).getroot().find('world')
    boats = [m for m in world.findall('model') if m.get('name', '').startswith('usv_')]
    assert [b.get('name') for b in boats] == ['usv_azure', 'usv_marine']
    xa, ya = map(float, boats[0].find('pose').text.split()[:2])
    xm, ym = map(float, boats[1].find('pose').text.split()[:2])
    # Heading west at the readiness start: Marine 6 m to the right, i.e. north
    assert abs(xm - xa) < 1e-6 and abs(ym - ya - mcw.SPACING_M) < 1e-6
    assert all(i.findtext('name') != 'usv_boat' for i in world.findall('include'))
