#!/usr/bin/env python3
"""Compatibility facade for UVPack's standalone geometry companion."""
from pathlib import Path
import sys

# Source-checkout entry point. Installed consumers import uvpack_geometry directly.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from uvpack_geometry import legacy as _legacy
from uvpack_geometry.legacy import *  # Preserve the historical public helpers.


def __getattr__(name):
    return getattr(_legacy, name)


if __name__ == '__main__':
    sys.exit(_legacy.main(sys.argv[1:]))
