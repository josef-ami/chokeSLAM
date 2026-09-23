"""
Bench check for the camera (checkpoint D, section 15.8): the 180-degree
correction, CAMERA_BEARING_SIGN, CAMERA_HEIGHT_MM and the colour thresholds.

Put ONE pillar at a known bearing and range from the lens (e.g. 30 deg to the
robot's RIGHT, 500 mm), then:

    python3 camera_check.py --real --bearing 30 --range 500 --out check.png
    python3 camera_check.py --image frame.png --bearing 30 --range 500     # a saved RAW frame

It corrects the frame (color_id.correct_frame), draws the box color_id would
use for that bearing and range (plus the optical axis and the horizon), prints
the red / green fractions and the verdict, and saves the annotated image and
the raw frame (--out, and <out>_raw.png).
    box on the pillar                    -> sign, rotation, height are right
    box mirrored to the other side       -> CAMERA_BEARING_SIGN is wrong
    pillar upside-down / image inverted  -> CAMERA_ROTATE_180 is wrong
    box too high / too low               -> CAMERA_HEIGHT_MM is wrong
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2

import color_id
import config


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--real", action="store_true", help="capture one frame with Picamera2")
    src.add_argument("--image", help="a RAW frame saved earlier (as the camera delivers it)")
    ap.add_argument("--bearing", type=float, required=True, help="deg, + = robot's right, from the lens")
    ap.add_argument("--range", type=float, required=True, help="mm from the lens to the pillar's centre")
    ap.add_argument("--out", default="camera_check.png")
    a = ap.parse_args()

    if a.real:
        from camera_source import Picamera2Source
        cam = Picamera2Source()
        cam.start()
        t0 = time.monotonic()
        fr = None
        while time.monotonic() - t0 < 5.0 and cam.is_alive():
            fr = cam.get_latest_frame()
            if fr is not None and fr[2] >= 5:              # let exposure settle
                break
            time.sleep(0.05)
        cam.stop()
        if fr is None:
            sys.exit(f"no frame: {cam.error}")
        raw = fr[0]
    else:
        raw = cv2.imread(a.image)
        if raw is None:
            sys.exit(f"could not read {a.image}")
    cv2.imwrite(a.out.rsplit(".", 1)[0] + "_raw.png", raw)
    img = color_id.correct_frame(raw)
    face = a.range - color_id.PILLAR_HALF_MM
    roi, why = color_id.pillar_roi(a.bearing, face)
    K = config.CAMERA_K
    view = img.copy()
    cv2.line(view, (int(K[0][2]), 0), (int(K[0][2]), view.shape[0]), (255, 255, 0), 1)       # optical axis
    cv2.line(view, (0, int(K[1][2])), (view.shape[1], int(K[1][2])), (255, 255, 0), 1)       # horizon
    print(f"sign {config.CAMERA_BEARING_SIGN:+d}, rotate180 {config.CAMERA_ROTATE_180}, "
          f"height {config.CAMERA_HEIGHT_MM:.0f} mm")
    if roi is None:
        print(f"bearing {a.bearing:+.1f} deg, range {a.range:.0f} mm: {why}")
    else:
        colour, rf, gf = color_id.classify(img, roi)
        cv2.rectangle(view, (roi.u0, roi.v0), (roi.u1 - 1, roi.v1 - 1), (255, 0, 255), 2)
        print(f"bearing {a.bearing:+.1f} deg, range {a.range:.0f} mm: box u {roi.u0}-{roi.u1}, v {roi.v0}-{roi.v1}; "
              f"red {rf:.0%}, green {gf:.0%} -> {colour.upper() if colour else 'not confident'}")
    cv2.imwrite(a.out, view)
    print(f"saved {a.out} (annotated, corrected) and {a.out.rsplit('.', 1)[0]}_raw.png (raw)")


if __name__ == "__main__":
    main()
