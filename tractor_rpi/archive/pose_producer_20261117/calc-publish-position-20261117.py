#!/usr/bin/env python3
"""Field candidate: RTCM/GNSS compatibility publisher with IMU/encoder capture."""
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

from tractor_rpi.archive.pose_producer_20261117.calc_publish_position_20261117 import main

if __name__=="__main__":
    raise SystemExit(main())
