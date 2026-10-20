#!/usr/bin/env python3
"""Count YOLO labels and evaluate an explicitly selected detection model."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import yaml
from ultralytics import YOLO


def main():
    """라벨별 사진 수를 세고 모델을 평가해 점수와 혼동행렬을 JSON 파일로 남긴다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    data_path = args.data.resolve()
    config = yaml.safe_load(data_path.read_text())
    root = Path(config.get('path', '.'))
    if not root.is_absolute():
        root = (data_path.parent / root).resolve()
    config['path'] = str(root)
    names = config['names']
    if isinstance(names, list):
        names = dict(enumerate(names))
    model = YOLO(str(args.weights.resolve()))
    if dict(model.names) != names:
        raise ValueError(f'Model/dataset class mismatch: {model.names} != {names}')
    report = {
        'weights_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
        'classes': names,
        'collection_conditions': 'Distance/angle/lighting/occlusion metadata not recorded',
        'splits': {},
    }
    hashes = {}
    duplicates = []
    for split in ('train', 'val', 'test'):
        images_dir = root / config[split]
        labels_dir = images_dir.parent / 'labels'
        image_counts = Counter()
        object_counts = Counter()
        missing_labels = []
        images = sorted(p for p in images_dir.iterdir()
                        if p.suffix.lower() in {'.jpg', '.jpeg', '.png', '.bmp', '.webp'})
        for image in images:
            digest = hashlib.sha256(image.read_bytes()).hexdigest()
            if digest in hashes and hashes[digest]['split'] != split:
                duplicates.append({'first': hashes[digest], 'second': {'split': split, 'file': image.name}})
            else:
                hashes[digest] = {'split': split, 'file': image.name}
            label = labels_dir / f'{image.stem}.txt'
            if not label.is_file():
                missing_labels.append(image.name)
                continue
            present = set()
            for line in label.read_text().splitlines():
                if not line.strip():
                    continue
                class_id = int(line.split()[0])
                if class_id not in names:
                    raise ValueError(f'Unknown class ID {class_id}: {label}')
                object_counts[class_id] += 1
                present.add(class_id)
            image_counts.update(present)
        report['splits'][split] = {
            'images': len(images),
            'class_images': {name: image_counts[i] for i, name in names.items()},
            'class_objects': {name: object_counts[i] for i, name in names.items()},
            'missing_labels': missing_labels,
        }
    report['identical_images_across_splits'] = duplicates
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cache = (root / config['val']).parent / 'labels.cache'
    if cache.is_file():
        backup = args.output_dir / f'validation-labels-{time.time_ns()}.cache'
        cache.rename(backup)
        report['validation_cache_backup'] = str(backup)
    resolved_yaml = args.output_dir / 'resolved_data.yaml'
    resolved_yaml.write_text(yaml.safe_dump(config, sort_keys=False))
    metrics = model.val(data=str(resolved_yaml.resolve()), split='val', imgsz=640,
                        conf=0.25, iou=0.45, device=args.device, workers=0,
                        project=str(args.output_dir.resolve()), name='validation',
                        exist_ok=True, plots=True)
    report['validation'] = {
        'metrics': {key: float(value) for key, value in metrics.results_dict.items()},
        'per_class': {
            names[int(i)]: {
                'precision': float(metrics.box.p[j]),
                'recall': float(metrics.box.r[j]),
                'mAP50': float(metrics.box.ap50[j]),
                'mAP50_95': float(metrics.box.ap[j]),
            } for j, i in enumerate(metrics.box.ap_class_index)
        },
        'confusion_matrix_rows_predicted_columns_actual': metrics.confusion_matrix.matrix.tolist(),
        'confusion_matrix_order': list(names.values()) + ['background'],
        'conf': 0.25, 'iou': 0.45,
    }
    output = args.output_dir / 'audit.json'
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
