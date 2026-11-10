"""
Boat profiles: what each of the two boats has (RoboBoat 2027).

Azure and Marine share the hull, thrusters, flight controller, GPS and software; they
differ in the mission computer and the payload (usv_nodes/config/boats/<name>.yaml). The
mission asks a profile what the boat can do (`can`), e.g. a water delivery goes to the boat
with the water cannon.
"""

from dataclasses import dataclass, field

CAPABILITIES = ('water', 'balls', 'lidar', 'turret_camera')


@dataclass(frozen=True)
class Camera:
    name: str
    model: str
    mount: str = 'hull'           # hull: fixed, looks ahead; turret: turns with the turret
    depth: bool = True


@dataclass(frozen=True)
class BoatProfile:
    name: str                     # also the ROS namespace
    number: int                   # the boat's number on the GCS link
    computer: str
    cameras: tuple = field(default_factory=tuple)
    lidar: bool = False
    water_cannon: bool = False
    balls: int = 0
    ip: str = ''                  # on the Ubiquiti network (GCS link)
    udp_port: int = 0             # the boat listens for the GCS here

    def can(self, capability):
        """Return whether the boat has the capability (one of CAPABILITIES)."""
        if capability not in CAPABILITIES:
            raise ValueError(f'unknown capability {capability!r}')
        return {'water': self.water_cannon,
                'balls': self.balls > 0,
                'lidar': self.lidar,
                'turret_camera': any(c.mount == 'turret' for c in self.cameras)}[capability]

    def capabilities(self):
        return tuple(c for c in CAPABILITIES if self.can(c))


def load(data):
    """Build a BoatProfile from a profile file's content (a dict); unknown keys are errors."""
    data = dict(data)
    cameras = tuple(Camera(**c) for c in data.pop('cameras', ()))
    if not cameras:
        raise ValueError('a boat needs at least one camera')
    names = [c.name for c in cameras]
    if len(set(names)) != len(names):
        raise ValueError(f'camera names repeat: {names}')
    for c in cameras:
        if c.mount not in ('hull', 'turret'):
            raise ValueError(f'camera {c.name}: mount {c.mount!r} is not hull or turret')
    profile = BoatProfile(cameras=cameras, **data)
    if profile.number < 1 or profile.balls < 0:
        raise ValueError('number starts at 1 and balls cannot be negative')
    return profile
