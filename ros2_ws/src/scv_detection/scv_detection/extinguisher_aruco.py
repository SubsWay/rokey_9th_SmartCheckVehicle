"""소화기 검출 영역 안의 ArUco 마커를 읽는 계산·표시 함수 모음.

카메라 구독, 압축 영상 복원, YOLO 추론, ROS 실행과 로그 출력은 매니저가 담당한다.
DICT_4X4_1000의 ID 0·1·2에는 독립 test.py와 같은 임시 제조 정보를 사용한다.
더미 만료 상태는 실제 압력 상태와 별개이며 실제 DB나 불량 토픽에는 연결하지 않는다.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import date, datetime
from zoneinfo import ZoneInfo

import cv2
import numpy as np


DICTIONARY_NAME = 'DICT_4X4_1000'
# 실제 DB 데이터를 전달하기 전까지 사용할 예시이며, 만료일 당일까지 유효하다고 가정한다.
DUMMY_DATA = {
    0: {'manufacture_date': '2010-03-01', 'manufacture_number': 'DUMMY-FE-0000', 'expiry_date': '2020-12-31'},
    1: {'manufacture_date': '2012-07-15', 'manufacture_number': 'DUMMY-FE-0001', 'expiry_date': '2024-12-31'},
    2: {'manufacture_date': '2025-01-10', 'manufacture_number': 'DUMMY-FE-0002', 'expiry_date': '2035-12-31'},
}


class ExtinguisherAruco:
    """소화기 좌표를 입력받아 그 안의 마커를 읽고 결과를 반환하는 도우미다."""

    def __init__(self, records=None):
        """ArUco 검출기를 준비한다. records는 ID별 제조 정보 사전이며 생략하면 더미 정보를 쓴다.

        사진을 받거나 ROS 노드를 만들지 않는다. OpenCV의 ArUco 기능이 없으면 오류로 알린다.
        """
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'ArucoDetector'):
            raise RuntimeError('ArUco requires cv2.aruco.ArucoDetector (OpenCV 4.7 or later)')
        self.records = deepcopy(DUMMY_DATA if records is None else records)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    @staticmethod
    def clip_bbox(bbox, frame_shape):
        """소화기 좌표와 사진 크기를 받아 사진 안에 맞춘 (x1, y1, x2, y2)를 반환한다.

        좌표가 잘못됐거나 사진 안에 남는 영역이 없으면 None을 반환한다.
        """
        try:
            values = np.asarray(bbox, dtype=float)
        except (TypeError, ValueError):
            return None
        if values.shape != (4,) or not np.isfinite(values).all() or len(frame_shape) < 2:
            return None
        height, width = frame_shape[:2]
        if width <= 0 or height <= 0 or values[0] >= values[2] or values[1] >= values[3]:
            return None
        x1 = max(0, min(width, int(np.floor(values[0]))))
        y1 = max(0, min(height, int(np.floor(values[1]))))
        x2 = max(0, min(width, int(np.ceil(values[2]))))
        y2 = max(0, min(height, int(np.ceil(values[3]))))
        return (x1, y1, x2, y2) if x1 < x2 and y1 < y2 else None

    @staticmethod
    def _marker_inside(points, bbox):
        """마커의 네 꼭짓점과 소화기 좌표를 받아 꼭짓점 모두가 영역 안에 있으면 True를 반환한다."""
        x1, y1, x2, y2 = bbox
        return bool(np.all((points[:, 0] >= x1) & (points[:, 0] < x2)
                           & (points[:, 1] >= y1) & (points[:, 1] < y2)))

    def lookup_marker(self, marker_id, today):
        """마커 ID와 비교 날짜를 받아 제조 정보·더미 만료 상태를 반환한다.

        등록되지 않은 ID나 잘못된 제조 정보는 UNKNOWN으로 반환한다.
        날짜는 YYYY-MM-DD 문자열이고 today는 datetime.date다.
        """
        record = {'marker_id': int(marker_id), 'dictionary': DICTIONARY_NAME,
                  'manufacture_date': None, 'manufacture_number': None, 'expiry_date': None,
                  'expiry_status': 'UNKNOWN', 'days_remaining': None,
                  'reference_date': today.isoformat(), 'reason': 'ID has no manufacturing record'}
        data = self.records.get(marker_id)
        if data is None:
            return record
        try:
            # 입력 사전에 다른 필드가 있어도 검출 좌표나 ID를 덮어쓰지 않는다.
            for key in ('manufacture_date', 'manufacture_number', 'expiry_date'):
                record[key] = data[key]
            manufactured = date.fromisoformat(record['manufacture_date'])
            expires = date.fromisoformat(record['expiry_date'])
            if manufactured > today or manufactured > expires or not record['manufacture_number']:
                raise ValueError('manufacture date/number inconsistent with reference/expiry date')
            remaining = (expires - today).days
            record.update(expiry_status='EXPIRED' if remaining < 0 else 'VALID',
                          days_remaining=remaining,
                          reason='expiry date passed' if remaining < 0 else 'expiry date not passed')
        except (KeyError, TypeError, ValueError) as error:
            record['reason'] = f'invalid manufacturing record: {error}'
        return record

    def inspect(self, frame, extinguishers, today=None):
        """같은 사진의 소화기 영역만 검사해 소화기별 마커 결과 목록을 반환한다.

        frame은 원본 BGR 영상이며 extinguishers는 box_index·bbox·confidence를 담은 목록이다.
        소화기 목록이 비어 있으면 마커 검출을 실행하지 않고 빈 목록을 반환한다.
        bbox는 전체 영상 기준 xyxy다. 반환 마커의 corners도 전체 영상 좌표다.
        여러 소화기 영역에 동시에 포함되는 마커는 특정 소화기에 연결하지 않는다.
        """
        if not extinguishers:
            return []
        if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
                or frame.ndim != 3 or frame.shape[2] != 3):
            raise ValueError('frame must be a uint8 BGR image')
        today = today if today is not None else datetime.now(ZoneInfo('Asia/Seoul')).date()
        bodies = []
        for extinguisher in extinguishers:
            bbox = self.clip_bbox(extinguisher['bbox'], frame.shape)
            if bbox is not None:
                bodies.append({'box_index': int(extinguisher['box_index']), 'bbox': bbox,
                               'confidence': float(extinguisher['confidence'])})
        reports = []
        for body in bodies:
            report = dict(body, markers=[], reason='aruco marker not detected in extinguisher')
            reports.append(report)
            x1, y1, x2, y2 = body['bbox']
            try:
                # 전체 화면에서는 검출하지 않아 배경의 마커가 결과에 들어오지 않는다.
                gray = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)
            except cv2.error as error:
                report['reason'] = f'aruco detection failed: {error}'
                continue
            if ids is None:
                continue
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                points = marker_corners.reshape(4, 2).astype(float) + np.array([x1, y1])
                if not self._marker_inside(points, body['bbox']):
                    report['reason'] = 'aruco marker corners extend outside extinguisher box'
                    continue
                owners = [candidate for candidate in bodies
                          if self._marker_inside(points, candidate['bbox'])]
                if len(owners) != 1:
                    report['reason'] = 'aruco marker belongs to multiple extinguisher boxes'
                    continue
                marker = self.lookup_marker(int(marker_id), today)
                marker['corners'] = points.tolist()
                report['markers'].append(marker)
            if report['markers']:
                report['reason'] = 'aruco markers detected in extinguisher'
        return reports

    @staticmethod
    def draw_result(frame, reports):
        """매니저의 표시용 영상에 마커 테두리·ID·더미 만료 상태를 그려 같은 영상을 반환한다.

        빨강은 만료, 초록은 유효, 회색은 미확인이다. 이 색은 소화기 압력 판정과 별개다.
        reports는 inspect의 결과 목록이며 한글을 지원하지 않는 화면 글꼴에 맞춰 영어로 쓴다.
        """
        colors = {'EXPIRED': (0, 0, 255), 'VALID': (0, 255, 0), 'UNKNOWN': (192, 192, 192)}
        height, width = frame.shape[:2]
        for report in reports:
            for marker in report['markers']:
                points = np.rint(marker['corners']).astype(np.int32).reshape(4, 2)
                color = colors[marker['expiry_status']]
                cv2.polylines(frame, [points], True, color, 2, cv2.LINE_AA)
                cv2.circle(frame, tuple(points[0]), 4, color, -1, cv2.LINE_AA)
                label = f"ArUco ID {marker['marker_id']} {marker['expiry_status']} (DUMMY EXPIRY)"
                label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 2)[0][0]
                x = max(0, min(int(points[:, 0].min()), width - label_width - 5))
                y = int(points[:, 1].max()) + 20
                if y >= height - 5:
                    y = max(18, int(points[:, 1].min()) - 10)
                cv2.putText(frame, label, (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, label, (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2, cv2.LINE_AA)
        return frame

    @staticmethod
    def format_logs(reports):
        """소화기별 마커 결과를 받아 매니저가 info 로그로 출력할 한국어 문자열 목록을 반환한다."""
        logs = []
        labels = {'EXPIRED': '만료', 'VALID': '유효', 'UNKNOWN': '미확인'}
        for report in reports:
            owner = report['box_index']
            if not report['markers']:
                logs.append(f"ArUco 소화기박스={owner} | 미검출/미확인 | 사유={report['reason']}")
            for marker in report['markers']:
                logs.append(
                    f"[ArUco 더미] 소화기박스={owner} | ID={marker['marker_id']} | "
                    f"제조일자={marker['manufacture_date']} | 제조번호={marker['manufacture_number']} | "
                    f"만료일={marker['expiry_date']} | "
                    f"상태={labels[marker['expiry_status']]}({marker['expiry_status']}) | "
                    f"기준일={marker['reference_date']}")
                if marker['expiry_status'] == 'UNKNOWN':
                    logs.append(f"ArUco ID={marker['marker_id']} 미확인 사유: {marker['reason']}")
        return logs
