#!/usr/bin/env python3
"""관제가 저장한 칸 맵(cellmap_dir 의 Z*.json)을 지도 위 히트맵 PNG 로 그린다.

    source ros2_ws/install/setup.bash
    python3 ros2_ws/src/scv_manager/scripts/cellmap_heatmap.py                # ~/scv_cellmap → ./rssi_heatmap.png
    python3 ros2_ws/src/scv_manager/scripts/cellmap_heatmap.py -d <폴더> -o <파일.png>
    python3 ros2_ws/src/scv_manager/scripts/cellmap_heatmap.py --bag ~/scv_logs/bags/<회차>   # 실제 궤적도 겹침

숫자는 칸의 RSSI 중앙값[dBm], n 은 표본 수. 빨간 테두리는 음영, 회색은 미검출, 빈 칸은 미측정.
--bag 을 주면 그 bag 의 위치 토픽(기본 /robot3/amcl_pose)을 로봇의 실제 궤적으로 겹친다.
칸 맵을 파일로 쓸 때(backend:=file)만 된다.
"""
import argparse
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
from matplotlib import font_manager
from matplotlib.patches import Rectangle

from scv_manager.route_builder import wall_points
from scv_survey.cellmap_store import read_map_yaml

LOW, HIGH = -80.0, -40.0    # 색 범위 [dBm]


def main():
    default_map = os.path.join(get_package_share_directory('scv_bringup'), 'maps', 'testbed_v1.yaml')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('-d', '--dir', default='~/scv_cellmap', help='칸 맵 폴더 (기본 ~/scv_cellmap)')
    parser.add_argument('-m', '--map', default=default_map, help='지도 yaml')
    parser.add_argument('-o', '--out', default='rssi_heatmap.png', help='저장할 PNG')
    parser.add_argument('--bag', help='실제 궤적을 겹칠 rosbag2 폴더')
    parser.add_argument('--topic', default='/robot3/amcl_pose', help='궤적 토픽 (PoseWithCovarianceStamped)')
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(os.path.expanduser(args.dir), 'Z*.json')))
    if not files:
        raise SystemExit(f'{args.dir} 에 Z*.json 이 없습니다 (회차가 칸 확정까지 갔는지 확인)')

    korean = [f.fname for f in font_manager.fontManager.ttflist if 'Noto Sans CJK' in f.name]
    if korean:
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=korean[0]).get_name()

    walls = wall_points(read_map_yaml(args.map))
    xs, ys = [w[0] for w in walls], [w[1] for w in walls]
    fig, ax = plt.subplots(figsize=(6.2, 10.5), dpi=110)
    cmap = plt.get_cmap('RdYlGn')
    titles = []

    for path in files:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        size = data['map']['cell']
        ox, oy = data['map']['origin']
        titles.append(f"{data['zone']} run {data['run_id']}")

        for cell in data['cells']:
            x0, y0 = ox + cell['ix'] * size, oy + cell['iy'] * size
            if cell['state'] == 'miss' or cell['median'] is None:
                face, label = '#b0b0b0', '미검출'
            else:
                face, label = cmap((cell['median'] - LOW) / (HIGH - LOW)), f"{cell['median']:.0f}"
            shadow = cell['state'] == 'shadow'
            ax.add_patch(Rectangle((x0, y0), size, size, facecolor=face,
                                   edgecolor='#A32D2D' if shadow else 'white', lw=2 if shadow else 1))
            ax.text(x0 + size / 2, y0 + size / 2 + 0.06, label, ha='center', va='center', fontsize=8)
            ax.text(x0 + size / 2, y0 + size / 2 - 0.12, f"n={cell['n']}", ha='center', va='center',
                    fontsize=6, color='#333')

    ax.scatter(xs, ys, s=9, marker='s', color='#333', zorder=3)
    if args.bag:
        track = read_track(os.path.expanduser(args.bag), args.topic)
        if track:
            ax.plot([p[0] for p in track], [p[1] for p in track], color='#1f3a93', lw=1.6, alpha=0.85,
                    zorder=4, label=f'실제 궤적 ({args.topic}, {len(track)}점)')
            ax.plot(*track[0], 'o', color='#1f3a93', ms=6, zorder=5, label='시작')
            ax.plot(*track[-1], 'x', color='#1f3a93', ms=8, mew=2, zorder=5, label='끝')
            ax.legend(loc='upper left', fontsize=7, framealpha=0.9)
        else:
            print(f'{args.bag} 에 {args.topic} 이 없습니다')
    fig.colorbar(plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(LOW, HIGH)), ax=ax,
                 fraction=0.035, pad=0.02, label='RSSI 중앙값 [dBm]')
    ax.set_title('RSSI 칸 맵 — ' + ', '.join(titles)
                 + '\n숫자: 중앙값 dBm · n: 표본 수 · 빨간 테두리: 음영 · 회색: 미검출 · 빈 칸: 미측정', fontsize=9)
    ax.set_aspect('equal')
    ax.set_xlim(min(xs) - 0.2, max(xs) + 0.2)
    ax.set_ylim(min(ys) - 0.2, max(ys) + 0.2)
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(args.out)
    print(f'저장: {args.out} ({", ".join(titles)})')


def read_track(bag, topic):
    """bag 의 PoseWithCovarianceStamped 토픽을 [(x, y)] 로."""
    import rosbag2_py
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    track = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        pose = deserialize_message(data, PoseWithCovarianceStamped).pose.pose.position
        track.append((pose.x, pose.y))
    return track


if __name__ == '__main__':
    main()
