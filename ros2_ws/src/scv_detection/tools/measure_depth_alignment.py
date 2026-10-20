"""RGB와 Depth(compressedDepth)가 화면에서 얼마나 어긋나는지 재고 보정값을 구하는 도구.

사용법: 구역 위에 상자를 두고 로봇을 세운 뒤
  python3 tools/measure_depth_alignment.py --ns /robot4 --rect 177,273,300,398 --rect 350,438,335,360
--rect 는 RGB 영상에서 눈으로 읽은 상자 사각형(왼쪽,오른쪽,위,아래 픽셀)이다. 한 개 이상 준다.
결과로 depth_x = scale_x*rgb_x + offset_x, depth_y = scale_y*rgb_y + offset_y 를 출력하며,
이 값을 매니저 파라미터 shutter_depth_pixel_scale_x/offset_x/scale_y/offset_y 에 넣는다.
2026-10-15 오전 robot4(카메라 설정 변경 전, 640x480) 측정값: scale_x 0.775, offset_x 79, scale_y 0.670, offset_y 85.
10:40 무렵 카메라 설정 변경(704x704) 뒤에는 거의 1:1이라 매니저 기본값은 1, 0, 1, 0(보정 없음)이다.
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from scv_detection.scv_detection_manager import decode_compressed_depth


def grab_pair(namespace, seconds=10.0):
    """RGB와 압축 Depth를 잠시 받아 촬영 시각이 가장 가까운 한 쌍(BGR 영상, 미터 Depth)을 돌려준다."""
    rclpy.init()
    node = Node('measure_depth_alignment')
    rgb, depth = [], []
    stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    node.create_subscription(CompressedImage, f'{namespace}/oakd/rgb/image_raw/compressed', rgb.append,
                             QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=30))
    node.create_subscription(CompressedImage, f'{namespace}/oakd/stereo/image_raw/compressedDepth', depth.append,
                             QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10))
    start = time.time()
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    if not rgb or not depth:
        raise SystemExit('RGB 또는 Depth 메시지를 받지 못했다')
    best = min(((abs(stamp(r) - stamp(d)), r, d) for r in rgb for d in depth), key=lambda item: item[0])
    frame = cv2.imdecode(np.frombuffer(best[1].data, np.uint8), cv2.IMREAD_COLOR)
    return frame, decode_compressed_depth(best[2]), best[0]


def closer_than_floor(depth_m, margin_m=0.12, skip_rows=230):
    """행마다 바닥 깊이(중앙값)보다 margin_m 이상 가까운 픽셀, 즉 바닥 위 물체 후보 마스크를 만든다."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    rows = np.array([np.median(r[v]) if np.any(v) else 0 for r, v in zip(depth_m, valid)])
    mask = valid & (depth_m < rows[:, None] - margin_m)
    mask[:skip_rows] = False
    return mask


def fit_mapping(xs, ys, rects, shape):
    """Depth 물체 픽셀을 RGB 사각형 안으로 가장 잘 보내는 축별 배율·이동을 격자 탐색으로 찾는다."""
    height, width = shape
    target = np.zeros((height, width), bool)
    for left, right, top, bottom in rects:
        target[top:bottom, left:right] = True

    def score(ax, bx, ay, by):
        u = np.rint((xs - bx) / ax).astype(int)
        v = np.rint((ys - by) / ay).astype(int)
        ok = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if ok.sum() < 100:
            return 0.0
        hit = target[v[ok], u[ok]]
        precision, recall = hit.mean(), hit.sum() / target.sum()
        return 2 * precision * recall / (precision + recall + 1e-9)

    best = max(((score(ax, bx, ay, by), ax, bx, ay, by)
                for ax in np.arange(0.5, 1.01, 0.02) for bx in range(0, 200, 4)
                for ay in np.arange(0.5, 1.01, 0.02) for by in range(0, 130, 4)), key=lambda item: item[0])
    _, ax0, bx0, ay0, by0 = best
    return max(((score(ax, bx, ay, by), ax, bx, ay, by)
                for ax in np.arange(ax0 - 0.03, ax0 + 0.031, 0.005) for bx in range(bx0 - 6, bx0 + 7)
                for ay in np.arange(ay0 - 0.03, ay0 + 0.031, 0.005) for by in range(by0 - 6, by0 + 7)),
               key=lambda item: item[0])


def main():
    """명령 인자를 읽고 한 쌍을 받아 보정값을 출력하며 확인용 겹침 그림을 저장한다."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='/robot4')
    parser.add_argument('--rect', action='append', required=True, help='left,right,top,bottom (RGB 픽셀)')
    parser.add_argument('--out', default='depth_alignment_check.jpg')
    args = parser.parse_args()
    rects = [tuple(int(v) for v in rect.split(',')) for rect in args.rect]
    frame, depth, difference = grab_pair(args.ns)
    print(f'pair time difference {difference * 1000:.0f} ms, depth shape {depth.shape}')
    mask = closer_than_floor(depth)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    keep = np.zeros_like(mask)
    for index in np.argsort(-stats[1:, 4])[:len(rects)] + 1:
        keep |= labels == index
    ys, xs = np.nonzero(keep)
    f1, ax, bx, ay, by = fit_mapping(xs, ys, rects, depth.shape)
    print(f'f1={f1:.3f}  depth_x = {ax:.3f}*rgb_x + {bx:.1f}   depth_y = {ay:.3f}*rgb_y + {by:.1f}')
    print(f'parameters: shutter_depth_pixel_scale_x:={ax:.3f} shutter_depth_pixel_offset_x:={bx:.1f} '
          f'shutter_depth_pixel_scale_y:={ay:.3f} shutter_depth_pixel_offset_y:={by:.1f}')
    height, width = depth.shape
    map_x = (bx + ax * np.arange(width)[None, :]).astype(np.float32).repeat(height, 0)
    map_y = (by + ay * np.arange(height)[:, None]).astype(np.float32).repeat(width, 1)
    warped = cv2.remap(np.where(np.isfinite(depth), depth, 0), map_x, map_y, cv2.INTER_NEAREST)
    overlay = frame.copy()
    near = closer_than_floor(warped, skip_rows=200)
    overlay[near] = (overlay[near] * 0.3 + np.array([0, 255, 255]) * 0.7).astype(np.uint8)
    for left, right, top, bottom in rects:
        cv2.rectangle(overlay, (left, top), (right, bottom), (255, 0, 255), 2)
    cv2.imwrite(args.out, overlay)
    print(f'overlay saved to {args.out}')


if __name__ == '__main__':
    main()
