"""Identity and implemented contracts shared with the native source candidate."""
import json
from importlib.resources import files


def build_info():
    """Return an owned metadata dictionary; native GPU support is build-specific."""
    return json.loads(files(__package__).joinpath('release.json').read_text(encoding='utf-8'))


__version__ = build_info()['version']
