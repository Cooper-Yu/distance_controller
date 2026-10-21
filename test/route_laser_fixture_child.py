#!/usr/bin/env python3
"""Test-only explicit guard injection into installed code on isolated localhost domain 193."""

import importlib.machinery
import importlib.util
import os
from pathlib import Path
import sys


def main():
    if os.getenv('ROS_LOCALHOST_ONLY') != '1' or os.getenv('ROS_DOMAIN_ID') != '193':
        raise RuntimeError('Fixture child requires localhost-only ROS domain 193')
    from ament_index_python.packages import get_package_prefix

    installed = Path(get_package_prefix('distance_controller')) / 'lib/distance_controller'
    sys.path.insert(0, str(installed))
    from route_laser_runtime import RouteLaserGuard, ShadowRouteLaserGuard

    loader = importlib.machinery.SourceFileLoader(
        'fixture_runner', str(installed / 'task6_route_runner')
    )
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
    loader.exec_module(module)

    class InjectedConsole(module.Console):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            guard = (
                ShadowRouteLaserGuard
                if os.getenv('LASER_FIXTURE_SHADOW') == '1'
                else RouteLaserGuard
            )
            self.command_guard = guard(self.runner, self.emit)

    module.Console = InjectedConsole
    return module.main()


if __name__ == '__main__':
    raise SystemExit(main())
