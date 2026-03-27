#!/usr/bin/env python3
import os
import sys

import yaml


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default = os.path.normpath(
        os.path.join(here, "..", "config", "target_escape_teleop.yaml")
    )
    path = sys.argv[1] if len(sys.argv) > 1 else default
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    d = float(data["mission"]["target_initial_distance_m"])
    print(d, end="")


if __name__ == "__main__":
    main()
