"""시험용 키 입력: 터미널에서 키를 누르면 network_map_engine 서비스를 부른다.

  s  측정 끝 → 미검출·음영 칸을 회피 마스크로 (RViz 에 장애물로 보임)   ~/finish_survey
  r  새 측정 시작 → 샘플·히트맵·회피 마스크를 모두 비움                  ~/reset_survey
  q  종료

키를 읽어야 하므로 launch 가 아니라 따로 터미널에서 띄운다:
  ros2 run scv_survey survey_keys
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

HELP = 's: 측정 끝(회피 마스크 표시) · r: 새 측정(모두 비움) · q: 종료'


class SurveyKeys(Node):
    def __init__(self):
        super().__init__('survey_keys')
        engine = self.declare_parameter('engine', '/network_map_engine').value
        self.key_clients = {
            's': self.create_client(Trigger, f'{engine}/finish_survey'),
            'r': self.create_client(Trigger, f'{engine}/reset_survey'),
        }

    def call(self, key):
        cli = self.key_clients[key]
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{cli.srv_name} 가 없습니다 — network_map_engine 이 떠 있는지 확인')
            return
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            self.get_logger().error(f'{cli.srv_name} 응답 없음 (5 s)')
        elif res.success:
            self.get_logger().info(res.message)
        else:
            self.get_logger().warn(f'실패: {res.message}')


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None):
    if not sys.stdin.isatty():
        print('survey_keys 는 키를 읽어야 해서 터미널에서 직접 띄워야 합니다: ros2 run scv_survey survey_keys')
        return 1
    rclpy.init(args=args)
    node = SurveyKeys()
    node.get_logger().info(HELP)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.1).lower()
            if key == 'q':
                break
            if key in node.key_clients:
                # cbreak 중에도 로그 줄바꿈은 그대로 나온다
                node.call(key)
            elif key and key not in '\r\n':
                node.get_logger().info(HELP)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
