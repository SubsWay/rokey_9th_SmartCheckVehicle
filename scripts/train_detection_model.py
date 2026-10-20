#!/usr/bin/env python3
"""Train with fresh label caches and explicit paths; bypass unrelated yolo CLIs."""

import argparse
from pathlib import Path
import time

import yaml
from ultralytics import YOLO


def main():
    """예전 라벨 임시 저장 파일을 백업하고, 현재 라벨로 모델을 다시 학습한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--freeze', type=int, default=10)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    data = args.data.resolve()
    config = yaml.safe_load(data.read_text())
    root = Path(config.get('path', '.'))
    if not root.is_absolute():
        root = (data.parent / root).resolve()
    config['path'] = str(root)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for split in ('train', 'val', 'test'):
        cache = (root / config[split]).parent / 'labels.cache'
        if cache.is_file():
            backup = args.output_dir / f'{split}-labels-{time.time_ns()}.cache'
            cache.rename(backup)
            print(f'Cache backed up: {backup}', flush=True)
    resolved = args.output_dir / 'resolved_data.yaml'
    resolved.write_text(yaml.safe_dump(config, sort_keys=False))
    YOLO(str(args.weights.resolve())).train(
        data=str(resolved.resolve()), project=str(args.output_dir.resolve()),
        name='yolov8n', epochs=args.epochs, patience=20, imgsz=640, batch=16,
        device=args.device, workers=4, seed=42, deterministic=True,
        optimizer='AdamW', lr0=0.0005, freeze=args.freeze,
        fliplr=0.0, flipud=0.0, degrees=2, translate=0.05, scale=0.1,
        hsv_h=0.0, hsv_s=0.15, hsv_v=0.15, mosaic=0.2, close_mosaic=10)


if __name__ == '__main__':
    main()
