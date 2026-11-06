"""
The boat's namespace in a recorded run.

Our nodes run in the boat's namespace (sim.launch.py boat:=azure|marine): a run recorded
now has /azure/fix, one recorded before (phase 6) has /usv/fix. The report tools name the
topics /usv/<name> and read any bag through `names`, which maps the bag's own topic names
onto those. No ROS needed.
"""

PREFIX = '/usv/'


def names(recorded, wanted, boat=None):
    """
    Return {topic name in the bag: /usv/ name} for the wanted /usv/ names.

    The boat is the namespace holding most of the wanted topics (usv for an old bag); give
    `boat` when a bag holds two boats with the same topics.
    """
    relative = {w[len(PREFIX):]: w for w in wanted}
    if boat is None:
        count = {}
        for name in recorded:
            ns, _, rest = name.lstrip('/').partition('/')
            if rest in relative:
                count[ns] = count.get(ns, 0) + 1
        if not count:
            return {}
        best = max(count.values())
        top = sorted(ns for ns, n in count.items() if n == best)
        if len(top) > 1:
            raise ValueError(f'the bag has the topics of {", ".join(top)}: say which boat')
        boat = top[0]
    return {f'/{boat}/{rest}': w for rest, w in relative.items()
            if f'/{boat}/{rest}' in recorded}
