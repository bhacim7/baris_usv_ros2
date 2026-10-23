#!/usr/bin/env python3
"""Turn a video into numbered contact sheets so that every frame can be reviewed.

Each tile is one frame with its index and timestamp in a strip ABOVE the image, so nothing
in the frame (e.g. a HUD) is covered. No frame is skipped.

Usage (inside the container):
    python3 /ws/tools/video_sheets.py INPUT.mp4 OUTPUT_DIR [--cols C --rows R --width W]

Defaults: 4 x 2 tiles of 320 px for portrait video, 3 x 3 tiles of 480 px for landscape.
"""

import argparse
import os
import sys

import cv2
import numpy as np

LABEL_H = 24


def write_sheet(path, tiles, cols, rows, tile_w, tile_h):
    sheet = np.zeros((rows * tile_h, cols * tile_w, 3), dtype=np.uint8)
    for k, tile in enumerate(tiles):
        r, c = divmod(k, cols)
        sheet[r * tile_h:(r + 1) * tile_h, c * tile_w:(c + 1) * tile_w] = tile
    cv2.imwrite(path, sheet, [int(cv2.IMWRITE_JPEG_QUALITY), 85])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('video')
    p.add_argument('out_dir')
    p.add_argument('--cols', type=int)
    p.add_argument('--rows', type=int)
    p.add_argument('--width', type=int)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f'Cannot open {args.video}')

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    reported = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # Size from the first decoded frame: phone videos carry a rotation flag, so the
    # container's width/height properties can be swapped relative to the real image.
    ok, first = cap.read()
    if not ok:
        sys.exit('No frames could be decoded')
    h0, w0 = first.shape[:2]
    portrait = h0 > w0
    cols = args.cols or (4 if portrait else 3)
    rows = args.rows or (2 if portrait else 3)
    tile_w = args.width or (320 if portrait else 480)
    tile_h = int(round(h0 * tile_w / float(w0)))
    print(f'{w0}x{h0} at {fps:.3f} fps, {reported} frames reported, '
          f'{reported / fps:.2f} s; sheets of {cols}x{rows} tiles, {tile_w}x{tile_h} px each')

    os.makedirs(args.out_dir, exist_ok=True)
    per_sheet = cols * rows
    tiles, index, sheets = [], 0, 0
    frame = first                      # already decoded; no seeking, so no frame is lost
    while frame is not None:
        if (tile_w, tile_h) == (frame.shape[1], frame.shape[0]):
            image = frame
        else:
            image = cv2.resize(frame, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
        tile = np.zeros((tile_h + LABEL_H, tile_w, 3), dtype=np.uint8)
        tile[LABEL_H:] = image
        cv2.putText(tile, f'#{index}  t={index / fps:.2f}s', (5, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
        index += 1
        if len(tiles) == per_sheet:
            sheets += 1
            write_sheet(os.path.join(args.out_dir, f'sheet_{sheets:04d}.jpg'),
                        tiles, cols, rows, tile_w, tile_h + LABEL_H)
            tiles = []
        ok, frame = cap.read()
        if not ok:
            frame = None
    if tiles:
        sheets += 1
        write_sheet(os.path.join(args.out_dir, f'sheet_{sheets:04d}.jpg'),
                    tiles, cols, rows, tile_w, tile_h + LABEL_H)

    print(f'Frames read: {index}. Sheets written: {sheets} in {args.out_dir}')


if __name__ == '__main__':
    main()
