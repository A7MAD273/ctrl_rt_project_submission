"""
Compatibility launcher alias for bicycle_sim.launch.py.
"""

import importlib.util
import os


def generate_launch_description():
    sim_launch = os.path.join(os.path.dirname(__file__), 'bicycle_sim.launch.py')
    spec = importlib.util.spec_from_file_location('bicycle_sim_launch', sim_launch)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()
