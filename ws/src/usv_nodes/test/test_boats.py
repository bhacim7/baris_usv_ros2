"""The boat profiles in config/boats load and say what each boat can do."""

import glob
import os

import pytest
from usv_core import boat
import yaml

BOATS = os.path.join(os.path.dirname(__file__), '..', 'config', 'boats')


def profile(name):
    with open(os.path.join(BOATS, name + '.yaml')) as f:
        return boat.load(yaml.safe_load(f))


def test_every_profile_loads_and_is_named_after_its_file():
    files = sorted(glob.glob(os.path.join(BOATS, '*.yaml')))
    names = [os.path.basename(f)[:-len('.yaml')] for f in files]
    assert names == ['azure', 'marine']
    profiles = [profile(n) for n in names]
    assert [p.name for p in profiles] == names
    assert sorted(p.number for p in profiles) == [1, 2]


def test_marine_delivers_and_azure_does_not():
    azure, marine = profile('azure'), profile('marine')
    assert azure.capabilities() == ()
    assert marine.capabilities() == ('water', 'balls', 'turret_camera')
    assert marine.balls == 3
    assert azure.computer == 'jetson_agx_thor' and marine.computer == 'jetson_agx_orin'


def test_a_wrong_profile_is_refused():
    good = {'name': 'x', 'number': 1, 'computer': 'c',
            'cameras': [{'name': 'nav', 'model': 'zed2i'}]}
    assert boat.load(good).can('water') is False
    with pytest.raises(TypeError):
        boat.load({**good, 'lidarr': True})
    with pytest.raises(ValueError):
        boat.load({**good, 'cameras': []})
    with pytest.raises(ValueError):
        boat.load({**good, 'cameras': [{'name': 'nav', 'model': 'zed2i', 'mount': 'mast'}]})
    with pytest.raises(ValueError):
        boat.load(good).can('fly')
