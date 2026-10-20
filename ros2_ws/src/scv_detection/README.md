# scv_detection

기본 실행은 카메라 미리보기다. 서비스 요청이 들어올 때만 YOLO·랙·방화셔터 검사 중
한 가지를 계속 실행한다. 같은 서비스를 다시 호출하면 중지하고 원본 카메라로 돌아온다.
실행 진입점은 `scv_detection_manager.py:main`, 클래스는 `ScvDetectionManager`,
노드 이름은 `detection_alert`다. 운영용 `InspectFacility` 액션 서버는 아직 구현하지 않았다.

## 파일 역할

```text
scv_detection/
├── scv_detection_manager.py  # 카메라·서비스·YOLO·불량 발행·GUI·셔터용 Depth/TF 수신
├── shutter_obstacle.py      # 빨간 구역·RANSAC 바닥·장애물 높이·한글 경고 표시
├── extinguisher_pressure.py  # 정상 영역 안의 바늘색 판독·연속 정상 확인
├── rack_loading.py           # YOLO 랙·박스 소속 연결·경계 초과·연속 확인
├── rack_conditioner.py       # 파이프 자동 검출·정상 기준 비교·랙 표시
├── extinguisher_aruco.py     # 소화기 안의 마커 ID·더미 제조 정보 조회
└── exit_sign.py              # 비상구 표시 면의 밝기·점등 연속 확인
```

매니저의 `annotate_yolo_result`가 모든 클래스의 박스·이름·신뢰도와 압력·적재 판독 결과를
표시하고 로그 문자열을 반환한다. 압력·랙 알고리즘은 각각 함수 모듈에 두며
영상 수신·전체 및 부분 사진의 모델 실행·ROS 통신은 매니저에서 관리한다.

## 기본 실행

```bash
ros2 run scv_detection scv_detection_manager
```

인자 없이 실행하면 `/robot4/oakd/rgb/image_raw/compressed`를 구독하고
카메라 창을 띄운다. 대기 중에는 YOLO 추론, 랙 파이프 검출, ROI 표시를 하지 않는다.
카메라가 아직 연결되지 않았으면 창에 영상을 기다린다는 안내를 표시한다.
셔터용 정렬 설정은 같은 이름공간의 OAK-D에서 자동 조회한다. 기본 robot4에서는 실행 인자 없이
매니저를 켜고 `inspect_shutter` 서비스만 호출한다. 조회 중에는 자동 확인 안내가 나온다.

설치된 `share/scv_detection/models/best.pt`를 자동으로 찾는다.
YOLO 모델은 첫 YOLO 서비스 요청에서 로딩하고 이후 재사용한다.
잘못된 모델 경로는 YOLO 요청을 실패시키며 카메라 실행은 계속된다.
화면 환경이 없으면 GUI만 끄고 영상 발행과 서비스는 유지한다.

다른 로봇은 이름공간을 바꾼다. 예:

```bash
ros2 run scv_detection scv_detection_manager --ros-args -r __ns:=/robot3
```

화면 없는 실행은 `-p show:=false`, 다른 모델은 `-p weights:=/path/to/model.pt`다.
`enable_yolo`, `rack_enabled`는 서비스 사용 허용 옵션이며 기본값은 둘 다 true다.
두 옵션을 모두 false로 설정한 카메라 전용 실행도 가능하다.

## 소화기 압력판과 노란 바늘 판독

YOLO 검사를 시작하면 검출된 소화기마다 다음 순서로 압력을 확인한다.

1. 전체 영상에서 소화기 안의 압력판 후보를 연결한다. 후보가 없으면 **같은 모델**을 소화기 crop에 한 번 더 적용한다.
2. 압력판 안의 둥근 테두리를 찾고, 기존 HSV 색 범위로 녹색·청록색 정상 구간과 노란 바늘색을 추출한다.
3. 녹색 구간의 양 끝으로 바늘이 가린 정상 영역도 채운다. RGB에서 R이 G·B보다 20 이상 높은 빨간 배경 픽셀은 제외한다. 주황빛 바늘색 픽셀은 빨간 배경으로 제외하지 않는다.
4. 정상 영역 안의 붙어 있는 바늘색 픽셀이 `pressure_needle_min_pixels`개 이상이면 `NORMAL`, 없거나 부족하면 `ABNORMAL`이다. 단일 색 잡음 픽셀만으로 정상 판정하지 않는다.
5. 같은 압력계에서 정상 후보가 **기본 0.5초 이상, 최소 3개의 새 프레임에 연속** 나타나야 최종 `NORMAL`을 확정한다. 두 조건을 모두 충족해야 한다. 비정상 후보는 즉시 반영한다.

바늘 각도는 계산하지 않는다. 각도 정보는 정상 영역을 그리는 데만 사용하며, 회전축과
테두리를 제외한 반지름 40~90% 영역에서 바늘색을 검사한다. 화면의 녹색 호는 정상 범위,
노란 점은 정상 영역 안에서 확인한 바늘색 덩어리의 중심이다. 로그에는 존재 여부와 픽셀 수,
`raw_verdict`(이번 사진 판정), 최종 `verdict`, 정상 확인 프레임 수·경과 시간을 표시한다.

정상 복귀를 기다릴 때는 직전 비정상을 유지하며 `NORMAL pending`을 표시한다.
처음 본 정상 후보는 확인을 마칠 때까지 미확인이다. 압력판·정상 영역이 미확인이면
직전 정상도 유지하지 않고 미확인으로 전환하며 연속 확인을 초기화한다.
소화기 박스 겹침과 계기 중심·크기·정상 구간으로 같은 대상을 일대일 연결한다.
대상 누락·연결 모호성·큰 위치/크기 변화·검사 시작/중지·영상/추론 실패 때 이력을 버린다.
`camera_timeout`(기본 2초)을 넘긴 영상 단절은 별도 타이머로 이력과 오래된 GUI 판정을 지운다.
촬영 시각이 있는 중복·역순 프레임은 확인에 세지 않는다. 촬영 시각이 0이면 수신 시각을 사용한다.
대상 번호는 영상 안의 짧은 연결 번호이며 실제 설비 ID가 아니다.

카메라에서 주황빛으로 보이는 바늘과 어두운 청록색 정상 눈금도 기본 색 범위에 포함한다.
JPEG 색 번짐을 고려해 H 하한을 최대 3만 낮추고 주변보다
밝기 V가 15 이상 높은 픽셀만 보완한다. 바늘의 방향·길이·끝점 모양에 따른 판정은 사용하지 않는다.
붉은 눈금의 각도 띠를 함께 찾고 띠 밖의 파란 인쇄 영역을 제외한다. 붉은 눈금 띠가 없으면
H 90을 넘는 청색 부분은 정상 눈금 후보에서 제외하여 파란 글씨만으로 정상 범위를 만들지 않는다.

압력판은 박스 면적의 90% 이상이 소화기 안에 있고 면적이 소화기의 절반 이하여야 연결된다.
실제 판독에는 소화기 박스 안의 픽셀만 사용한다. 여러 소화기에 걸친 후보나 서로 다른 압력계가
두 개 보이는 경우는 미확인이다. 작은 원·잘린 테두리·정상 영역 부재는 미확인으로 보류한다.
정상 영역을 찾았는데 바늘색이 보이지 않는 경우는 비정상이다. 촬영 실패로 바늘색이 사라져도 비정상으로 나올 수 있다.
둥근 테두리를 찾는 검사는 심한 비스듬한 촬영을 거르는 보조 조건이며 카메라 자세 보정은 수행하지 않는다.

소화기에 연결된 압력판의 `Normal`/`Abnormal_left`/`Abnormal_right` 이름은 위치 후보로만 사용한다.
해당 압력판의 기존 상태는 바늘 판정으로 대체하여 서로 모순되는 불량을 중복 발행하지 않는다.
소화기와 연결되지 않은 이전 상태 모델의 압력판·점등 클래스에는 기존 상태 처리를 적용한다.
객체 모델의 `pressure_gauge`, `box`, `exit_sign`, `rack` 이름만으로 정상·불량을 확정하지 않는다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| pressure_enabled | true | 소화기 내부 압력판 검사 사용 |
| pressure_min_gauge_diameter | 24픽셀 | 원본에서 필요한 최소 계기판 지름, 16 이상 |
| pressure_needle_min_pixels | 3픽셀 | 정상 영역 안에서 붙어 있어야 하는 최소 바늘색 픽셀 수 |
| pressure_normal_confirm_seconds | 0.5초 | 연속 정상 확인의 최소 시간, 양수 |
| pressure_normal_confirm_frames | 3 | 연속 정상 확인의 최소 새 프레임 수, 2 이상 |
| pressure_boundary_margin_deg | 5.0도 | 이전 실행 설정 호환용. 현재 존재 판정에는 사용하지 않음 |
| pressure_yellow_lower / upper | [8,40,70] / [35,255,255] | 주황빛·노란 바늘의 OpenCV HSV 범위 |
| pressure_green_lower / upper | [38,25,20] / [120,255,255] | 녹색·어두운 청록색 범위, 청색은 붉은 눈금 띠 안에서만 사용 |
| pressure_normal_start_deg / end_deg | -1.0 / -1.0 | 둘 다 -1이면 녹색만 사용. 두 각도를 지정하면 녹색 검출 실패 때 보정 범위를 사용 |

HSV의 H는 0~179, S·V는 0~255다. 모든 설정은 재시작 후 적용한다.
보정 범위는 실제 압력계의 녹색 양 끝을 같은 촬영 자세에서 측정해 입력해야 한다.
자동 녹색 검출이 성공하면 자동 범위를 우선한다. 정상 영역이 확정된 뒤 바늘색이 없으면 비정상이다.
계기판 중심과 바늘 회전축이 거의 일치하는 정면 영상이 필요하며, 실제 bar/MPa 수치를 환산하는 기능은 아니다.

기본 실행과 기존 `inspect_yolo` 서비스를 그대로 사용한다. 이전 YOLO 상태 처리만 사용하려면:

```bash
ros2 run scv_detection scv_detection_manager --ros-args -p pressure_enabled:=false
```

압력 불량의 `detail`에는 `source="yellow_needle_presence"`, 소화기 `bbox`, `gauge_bbox`,
`dial_center`, `needle_in_normal_range`, `needle_in_normal_pixels`,
`green_start_deg`, `green_end_deg`, `range_source`, `reason`이 포함된다.
이전 `needle_angle_deg` 키는 호환용으로 남기며 값은 항상 null이다.
`range_source`는 `green_pixels` 또는 `configured_angles`다. `green_start_deg/end_deg` 필드는
보정 옵션 사용 시에도 비교에 사용한 정상 범위의 양 끝을 의미한다. 신뢰도는 소화기 YOLO 검출값이며
바늘색 판독의 정확도 점수가 아니다. 정상·미확인은 불량 토픽에 발행하지 않는다.
`raw_status`, `raw_reason`, `pressure_track_id`, `inspection_id`, `normal_confirm_frames`,
`normal_confirm_elapsed_s`, `normal_confirm_required_s`로 이번 사진 판정과 정상 복귀 대기를 구분한다.
화면·불량 발행은 같은 확정 상태를 사용한다. 정상 복귀 대기 중 비정상 발행은 직전 확정
비정상을 유지한 결과이며, 그 사진의 정상 후보는 `raw_status`에 따로 기록한다.

기존 RGB/HSV 색 범위와 픽셀 존재 판정은 유지한다. 길이·폭·중심 연결 등의 바늘 형태 조건은
추가하지 않는다. JPEG 압축·조명으로 색 판정이 한두 프레임 흔들려도 즉시 초록색으로 바뀌지
않게 하는 변경이다. 시간 확인은 화질을 개선하는 기능이 아니므로 지속적인 색 오판까지 해결하지는 않는다.

## YOLO 랙·박스 경계로 적재 상태 확인

`inspect_yolo`를 켜면 같은 YOLO 추론 결과의 `rack`과 `box`로 적재 상태도 확인한다.
별도 모델·정상 기준 사진·고정 화면 ROI를 추가하지 않는다. `inspect_rack` 서비스의
기존 파이프/정상 사진 비교 기능은 별도로 유지한다. 새 기능은 `rack_loading_enabled`로 제어한다.

1. 각 박스를 박스 면적 대비 겹침 비율이 가장 높은 랙에 연결한다. 상위 두 랙의 비율 차이가
   기본 0.10 이하면 소속을 미확인으로 보류한다. 어떤 랙과도 겹치지 않는 박스는 소속 미확인이다.
2. 연결된 박스가 랙의 왼쪽·오른쪽·위·아래 경계를 넘는지 독립적으로 계산한다.
   양쪽이 동시에 벗어나도 모두 기록한다. 원본 영상의 소수점 좌표를 사용한다.
3. 기본 허용오차 0에서는 어느 경계라도 넘으면 불량 후보다. 필요하면 랙 너비/높이의
   비율로 허용오차를 설정할 수 있다. 경계 초과 픽셀 수와 비율은 그대로 기록한다.
4. 같은 랙·박스에서 후보가 기본 3개 새 프레임에 연속으로 나타나야 정상/불량을 확정한다.
   확인 중에는 회색 미확인이다. 다른 박스가 추가·누락되어도 계속 연결되는 박스의 횟수는 유지한다.
   새 대상·모호한 연결은 새로 확인하고 사라진 대상의 이력은 버린다.
5. 한 랙의 연결 박스 중 확정 불량이 하나라도 있으면 랙은 불량이다. 모든 연결 박스가
   확정 정상이고 같은 구성으로 연속 정상 확인을 마치면 랙을 정상으로 표시한다.
   불량 박스가 한 번 누락됐다는 이유로 바로 랙을 정상으로 바꾸지 않는다.
   박스가 검출되지 않으면 정상으로 추정하지 않는다.

이전 대상 연결은 현재·이전 검출이 서로 가장 좋은 IoU 후보일 때 이어 쓴다.
다른 박스와 조금 겹쳐도 최상위 후보가 뚜렷하면 연결을 유지하고, 상위 두 점수가
기본 0.10 이하 차이면 보류한다. 박스별 확인과 랙 정상 복귀 확인은 같은 사진에서 함께 센다.

랙·박스가 영상 가장자리에서 잘릴 가능성이 있으면 경계 판정을 미확인으로 보류한다.
검사 시작/중지·대상 누락·카메라 변경/단절·영상 변환/추론 실패·중복/역순 촬영 시각에서
확인 이력을 초기화하거나 관측을 보류한다. 대상 번호는 짧은 영상 추적 번호이며 실제 설비 ID가 아니다.

정상은 초록색, 불량은 빨간색, 미확인은 회색이다. 초과 부분에는 `LEFT`, `RIGHT`, `TOP`,
`BOTTOM`과 초과 픽셀 수를 표시한다. 로그의 `raw_verdict`, `loading_frames`, `reason`으로
프레임 후보와 최종 상태를 구분한다. 화면과 발행은 같은 확정 상태를 사용한다.
미확인 박스에는 `confirm 1/3 matched` 또는 `confirm 1/3 new/ambiguous`처럼 진행 상황을 표시한다.
로그에는 `box_track`, `tracking`, 랙 정상 확인 횟수도 담는다. `Overflow RIGHT=...`는
경계 초과 후보가 계산됐다는 뜻이며, 최종 상태가 미확인이면 연속 확인 횟수와 보류 이유를 함께 본다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| rack_loading_enabled | true | YOLO 검사 중 랙·박스 적재 판독 사용 |
| rack_loading_tolerance_ratio | 0.0 | 랙 너비/높이 대비 경계 허용오차, 0 이상 0.5 미만 |
| rack_loading_association_margin | 0.10 | 상위 두 랙의 박스 겹침 비율 차이가 이 값 이하면 소속 보류 |
| rack_loading_confirm_frames | 3 | 같은 후보의 최소 연속 새 프레임 수, 1 이상 |
| rack_loading_match_iou | 0.30 | 이전 랙·박스와 같은 대상으로 연결할 최소 IoU |
| rack_loading_match_margin | 0.10 | 연결 후보의 상위 두 IoU 차이가 이 값 이하면 연결 보류 |

설정은 노드 재시작 후 적용한다. 기본값은 실제 영상으로 검증하지 않은 초기 설정이다.
불량은 `inspect/defects`에 `KIND_RACK=4`, `ABNORMAL=4`로 랙당 한 건 발행한다.
`detail.source="yolo_rack_box_containment"`, `rack_bbox`, `rack_track_id`, `inspection_id`,
`tolerance_ratio`, `protrusions`를 담는다. 각 불량 박스에는 `bbox`, `box_track_id`,
`directions`, `overflow_px`, `overflow_ratio`, `association_overlap_ratio`, 연속 확인 횟수를 기록한다.
좌표 형식은 랙·박스 모두 `[x1,y1,x2,y2]`이고 `coordinate_format="xyxy"`로 명시한다.
정상·미확인은 화면/로그로 확인하며 불량 전용 토픽에 발행하지 않는다.

랙 라벨은 적재물까지 함께 커지는 박스가 아니라 랙 구조 외곽을 나타내야 한다.
이 판정은 화면의 사각형 경계 초과이며 실제 앞뒤 돌출 거리·허용 적재 하중을 측정하지 않는다.
이번 구현은 테스트·추론·ROS 실행 없이 코드 구문과 변경 내역만 확인했다.

## 방화셔터 RANSAC 장애물 검사

`inspect_shutter`를 호출하면 RGB에 정렬된 Depth·CameraInfo·TF로 빨간 구역의 바닥을 추정한다.
장애물이 확정되면 좌측 상단에 `경고)장애물을 치워주십시오`, 정상으로 확정되면
`정상)장애물이 없습니다`를 표시한다. 입력 부족·구역 잘림·확인 중에는 미확인 문구를 표시한다.

기본 이름공간은 `/robot4`이며 실행 인자는 필요 없다. 정렬 켜짐과 정렬 대상 카메라 번호를
`/robot4/oakd/get_parameters`에서 자동 확인한다. robot3을 선택할 때만 `-r __ns:=/robot3`을 지정한다.
카메라·Depth·TF 주소와 정렬 조회 주소도 해당 이름공간으로 바뀐다.
기존 화면 클릭용 Depth 기능은 사용하지 않는다.

셔터 검사 시작·중지 예시:

```bash
ros2 service call /robot4/detection_alert/inspect_shutter std_srvs/srv/Trigger "{}"
```

필요한 입력, 한글 글꼴, TF 설정과 초기 감도는 [실행 안내](docs/shutter_obstacle.md)를 참고한다.

## 서비스 호출

노드 실행 터미널을 유지하고 다른 터미널에서 호출한다.

YOLO 검사:

```bash
ros2 service call /robot4/detection_alert/inspect_yolo std_srvs/srv/Trigger "{}"
```

랙 검사:

```bash
ros2 service call /robot4/detection_alert/inspect_rack std_srvs/srv/Trigger "{}"
```

YOLO 요청에서는 YOLO만, 랙 요청에서는 랙만 실행한다.
다른 로봇을 쓰면 서비스 주소의 robot4도 해당 이름공간으로 바꾼다.
같은 서비스를 처음 호출하면 시작, 다시 호출하면 중지한다. 자동 종료 시간은 없다.
다른 검사를 시작하려면 실행 중인 검사의 서비스를 다시 호출해 먼저 중지한다.
다른 모드가 실행 중이거나 모델 로딩·기준 변경 중이면 `success=false, message="busy: …"`로 응답한다.
기준 저장·초기화도 검사 중에는 거절한다.

| 서비스 | 대기 중 호출 | 같은 검사 실행 중 호출 |
|---|---|---|
| inspect_yolo | YOLO 시작 | YOLO 중지 |
| inspect_rack | 랙 검사 시작 | 랙 검사 중지 |

응답의 success는 시작·중지 성공 여부다. message의 `started`/`stopped`로 확인한다.
서비스는 최종 판정을 기다리지 않는다. 실제 검출·랙 판정은 영상과 로그로 표시하고,
불량만 아래 `inspect/defects` 토픽으로 발행한다.
랙은 기본 3프레임 연속 판정 후 확정한다. 첫 YOLO 시작은 모델 로딩 시간이 걸릴 수 있다.
카메라가 없거나 최근 2초 동안 유효 영상이 없으면 시작 요청을 실패시킨다.
중지 요청은 카메라가 끊겼거나 기준 사진이 없어도 처리한다.
영상 유효 시간은 `camera_timeout`으로 바꾸며 예전 duration/inspect_timeout 파라미터는 제거했다.

## 불량 판정 토픽 확인

기본 토픽은 `/robot4/inspect/defects`, 메시지 타입은 `scv_msgs/msg/InspectionResult`다 (VOLATILE).
`/robot4/inspect/result` 는 관제가 요청한 설비 점검(IF-05 `inspect_facility`)의 최종 판정 전용이다 (TRANSIENT_LOCAL, 회차 · 작업 · 설비 id 포함).
두 가지를 한 토픽에 QoS 를 달리해 보내면 관제 구독에 「incompatible QoS」 경고가 나서 10/16 에 나눴다.
켜져 있는 검사 모드에서 불량이 검출되면 서비스 중지나 로그 주기를 기다리지 않고 발행한다.

| 모드 | 발행 조건 | kind | verdict |
|---|---|---:|---:|
| YOLO | 소화기 압력계 정상 영역 안에 바늘색이 없음, 또는 해당 비정상에서 정상 복귀 확인 중 | 1: KIND_EXTINGUISHER | 4: ABNORMAL |
| YOLO | 소화기에 연결되지 않은 Abnormal_left 또는 Abnormal_right | 1: KIND_EXTINGUISHER | 4: ABNORMAL |
| YOLO | Exit_sign_off | 3: KIND_EXIT_LIGHT | 4: ABNORMAL |
| YOLO | 연결된 박스의 랙 경계 초과가 연속 확인됨 | 4: KIND_RACK | 4: ABNORMAL |
| 랙 | 연속 확인을 통과한 돌출 판정 | 4: KIND_RACK | 4: ABNORMAL |

- YOLO는 불량 물체마다 한 건, 랙은 프레임의 돌출 목록을 묶어 한 건 발행한다.
- 불량이 계속 보이면 각 처리 프레임마다 반복 발행한다. 동일 물체의 중복 제거 기능은 없다.
- 대기·중지·정상·미확인·검출 0건·추론 오류에는 불량 메시지를 보내지 않는다.
- 랙은 기본 3프레임 확인 직후 발행한다. 첫 돌출 프레임부터 발행하려면 노드 실행 시 `-p rack_confirm_frames:=1`을 설정한다. 오검출 억제 효과는 줄어든다.
- `stamp`는 입력 영상 시각이며 값이 0이면 현재 ROS 시각이다. `robot_id`는 이름공간에서 얻는다.
- `detail`은 모드·원인을 담은 JSON 문자열이다. YOLO는 클래스·신뢰도·bbox, 랙은 원인·좌우 돌출 목록을 담는다.
- 좌표는 전체 입력 사진 기준이다. YOLO 적재 판독의 랙·박스 bbox는 `[x1,y1,x2,y2]`다.
  기존 사진 비교 랙 서비스의 돌출 bbox는 `[x,y,width,height]`다.
- Trigger 요청에는 순찰·설비 식별자가 없으므로 `run_id=0`, `task_id`·`facility_id`·`image_ref`는 빈 값이다. `attempts=1`이고 메시지의 `image`는 비워 둔다. 영상은 기존 compressed 출력에서 확인한다.
- 현재 토픽은 **불량 검출 이벤트**다. 중지·정상 복귀의 해제 이벤트는 보내지 않으며, 메시지가 없다고 정상으로 판단하면 안 된다.
- QoS는 RELIABLE / KEEP_LAST / depth 10 / VOLATILE이다. 늦게 구독한 노드에 과거 불량을 재전송하지 않는다.

노드를 실행한 뒤 다른 터미널에서 전체 메시지를 본다.

```bash
ros2 topic echo /robot4/inspect/defects scv_msgs/msg/InspectionResult
```

불량 원인만 간단히 보려면:

```bash
ros2 topic echo /robot4/inspect/defects scv_msgs/msg/InspectionResult --field detail --full-length
```

한 건만 확인하거나 토픽 정보를 확인하려면:

```bash
ros2 topic echo /robot4/inspect/defects scv_msgs/msg/InspectionResult --once
ros2 topic info /robot4/inspect/defects
```

`echo`는 검사 시작 후 불량이 검출될 때까지 기다린다. 다른 로봇을 쓰면 주소의 robot4를 변경한다.
새 코드 적용에는 빌드·환경 source 후 실행 중인 노드의 재시작이 필요하다.

### 확인 현황 (2026-10-13)

아래는 이전 구현의 확인 기록이다. 2026-10-14의 연속 정상 확인 수정은 코드 구문과 변경 내역만
확인했으며, 테스트·추론·ROS 노드 실행은 사용자 요청에 따라 하지 않았다.

불량 발행을 포함한 선택 단위·회귀 시험 65개와 패키지 symlink 빌드 성공을 확인했다.
Depth 클릭 기능 추가 후 세 테스트 파일의 전체 시험은 109개가 통과했다.
별도 ROS 도메인의 합성 YOLO 검출·랙 JPEG 시험에서 불량 메시지 수신, 실제 echo 명령,
검사 중지 후 새 불량 발행 차단을 확인했다.
추가로 현재 실행 환경에서 불량 토픽 확인이 잘 된다는 사용자 보고를 받았다.
이 기록은 기능 동작 확인이며 클래스별 검출 정확도나 장시간 안정성의 정량 검증과는 구분한다.
Depth는 합성 16UC1/32FC1을 실제 ROS로 수신하여 클릭 콜백에서 1.250m/2.500m 로그가 `/rosout`에 전달되는 것까지 확인했다.
실제 robot4의 읽기 전용 구독에서도 RGB·Depth가 640×480, 같은 optical frame_id이며 Depth가 16UC1인 것을 확인했다.
실물 클릭 거리의 정확도나 실제 창의 클릭 체감 동작은 별도 사용자 확인 대상이다.

노란 바늘 판독 추가 후 네 테스트 파일의 전체 시험은 168개가 통과했고 패키지 symlink 빌드가 성공했다.
현재 5개 객체 클래스 `best.pt`의 CPU 추론·압력판 연결·판독 호출도 설치된 패키지에서 확인했다.
초기 구현 당시 기존 검증 사진의 정답 압력판 21개를 기본 설정으로 읽으면 20개는 바늘 근거 부족,
1개는 둥근 테두리 미검출로 모두 미확인이다. 이 자료에서 실물 압력 판독 성공을 확인한 것은 아니며,
현장 적용에는 노란 바늘과 녹색이 선명한 근접 정면 영상으로 색상·관측 조건을 확인해야 한다.

사용자 첨부 화면의 미확인 문제를 수정한 뒤 전체 189개 시험이 통과했다.
바늘 색상 H 하한을 15에서 8로 낮추고 어두운 청록색 눈금을 붉은 눈금 띠 안에서 찾도록 보완했다.
짧게 보이는 바늘은 중심 방향·길이·가늘기·끝점 방향의 조건을 함께 확인한다.
첨부 화면을 기존 카메라 비율로 되돌린 압력판에서는 바늘 약 130도·정상 구간 약 71~96도로
범위 밖을 판독했고, 이 사진을 JPEG 품질 70·85·95로 다시 압축한 수신 콜백 경로의 불량 발행도 확인했다.
회전된 계기판과 아래쪽 파란 인쇄 영역을 정상 눈금으로 읽지 않는 경우도 시험에 포함했다.
이 재현 자료는 화면에서 추출한 사진이며 카메라 원본의 실측 압력 정답을 검증한 것은 아니다.
자료 생성 방법은 `test/data/README.md`에 기록했다. 노드를 재시작해야 새 기본값이 적용되며,
이전 HSV 값을 실행 인자로 지정했다면 새 범위로 맞추거나 해당 인자를 생략한다.

## 랙 기준 준비

기존 정상 기준 NPZ가 있으면 시작 시 로드한다.
없거나 카메라 위치가 달라졌다면 대기 화면에서 다음처럼 준비한다.

1. 상자를 모두 랙 안에 넣고, 랙의 흰색 좌우 파이프가 ROI 안에 보이게 한다.
2. 영상 창에서 N을 눌러 정상 기준을 저장한다. 네 모서리 클릭은 필요 없다.
3. 상자를 옆으로 움직인 뒤 랙 서비스를 호출한다.
4. R은 메모리 기준만 초기화하며 저장 파일은 삭제하지 않는다.

기본 ROI 비율은 (x1=0.25, y1=0.30, x2=0.75, y2=0.83)이다.
필요하면 `rack_roi_x1/y1/x2/y2`를 조절한다.
ROI 테두리와 검사 범위는 랙 검사 중에만 표시된다.
`rack_reference_path`로 저장 파일을 명시할 수 있다.
기본 경로는 설치 위치에서 찾은 소스 `config/rack_reference.npz`이며,
소스 config 폴더가 없으면 기준을 메모리에 저장한다.

랙 비교는 BGR 최대 채널 차이를 사용한다.
기본 `rack_margin=30`, `rack_zone_width=90`은 경계 바깥 30~90픽셀 구간을 뜻한다.
`rack_diff_threshold=35`, `rack_min_area=300`, `rack_confirm_frames=3`이다.
사람·그림자·조명 변화도 검출될 수 있으며 실제 앞뒤 돌출 거리는 측정하지 않는다.

## 현재 모델과 색상

2026-10-13 실제 `best.pt` 로딩에서 다음 5개 객체 클래스를 확인했다.
역할과 이전 상태 색상은 클래스 **이름**으로 선택하므로 ID 순서가 바뀌어도 압력계와 박스를 혼동하지 않는다.

| 실제 ID | 클래스 | 기본 표시 |
|---:|---|---|
| 0 | box | 랙 연결·적재 판독에 따라 초록·빨강·회색 |
| 1 | exit_sign | 회색·미확인, 이름만으로 점등 상태를 판단하지 않음 |
| 2 | fire_extinguisher | 압력 판독에 따라 초록·빨강·회색 |
| 3 | pressure_gauge | 소화기와 연결되면 바늘 판독 결과, 단독 검출은 미확인 |
| 4 | rack | YOLO 적재 판독에 따라 초록·빨강·회색 |

### 이전 6개 상태 클래스 모델 호환

이전 상태 모델의 이름은 `YOLO_LABEL_STYLE_IDS`를 통해 `YOLO_CLASS_STYLES`의 색상표와 연결한다.
알 수 없는 이름은 회색으로 표시한다. 아래 표는 바늘 판독으로 대체하기 전의 기본 상태다.

| ID | 클래스 | 색상 | 로그 상태 |
|---:|---|---|---|
| 0 | Abnormal_left | 빨강 | ABNORMAL |
| 1 | Abnormal_right | 빨강 | ABNORMAL |
| 2 | Fire Extinguisher | 회색 | UNCONFIRMED |
| 3 | Normal | 초록 | NORMAL |
| 4 | Exit_sign_on | 초록 | NORMAL |
| 5 | Exit_sign_off | 빨강 | ABNORMAL |

전체 소화기 검출만으로 정상이라고 판단하지 않는다. 연결된 압력판을 실제로 판독한 결과로
소화기 상태를 표시한다. 여러 프레임의 압력 판정 안정화·대상 추적은 아직 수행하지 않는다.

## 스레드와 입출력

메인 스레드는 GUI·N/R 키를 처리하고, 백그라운드 executor의 세 워커는
영상 처리·서비스 시작 및 중지·겹치는 요청 응답을 담당한다.
상태 잠금과 검사 잠금으로 요청을 분리하고 끝난 요청의 늦은 결과는 버린다.

입력: `oakd/rgb/image_raw/compressed`.
출력: `fire_extinguisher/detections/compressed`.
로봇 이름공간이 각 토픽 앞에 붙는다. 출력 header는 입력에서 복사한다.
현재 QoS는 RELIABLE / KEEP_LAST / depth 1이다.
Depth는 별도 콜백 그룹에서 BEST_EFFORT / KEEP_LAST / depth 1로 구독한다.
불량은 `inspect/defects`에 `InspectionResult`로 발행한다 (설비 점검 최종 판정은 `inspect/result`). `Alert` 및 운영용 액션 연동은 아직 구현하지 않았다.

## 설치

OpenCV·NumPy·Torch 및 ROS의 cv_bridge 의존성을 선언했다. Ultralytics는 기본 rosdep에 키가
없어 함께 제공한 `rosdep.yaml`을 등록한다. 저장소 루트에서 실행한다.

```bash
printf 'yaml file://%s/ros2_ws/src/scv_detection/rosdep.yaml\n' "$PWD" \
  | sudo tee /etc/ros/rosdep/sources.list.d/10-scv-detection.list
rosdep update
rosdep install --from-paths ros2_ws/src/scv_detection ros2_ws/src/scv_msgs --ignore-src -r -y
```

`setup.py`에도 `numpy`, `opencv-python`, `torch`, `ultralytics` 의존을 선언했다.
colcon의 Python과 실행 환경의 Python에 의존성이 설치되어 있어야 한다.
GPU용 PyTorch를 쓰는 PC는 기존 CUDA 환경을 유지한다.

## 빌드와 단독 launch

```bash
cd ~/final_ws/ros2_ws
colcon build --symlink-install --packages-select scv_detection
source install/setup.bash
ros2 run scv_detection scv_detection_manager
```

단독 launch도 같은 서비스 구조를 사용한다.

```bash
ros2 launch scv_detection detection_alert.launch.py
```

통합 bringup 변경은 별도 통합 작업에서 진행한다.

## 이전 5클래스 모델 학습 기록

아래 학습 수치와 해시는 이전 5개 상태 클래스 모델의 기록이다. 현재 5개 객체 클래스 모델과 구분한다. 10/12에 기존 라벨 캐시를 백업하고
같은 277장 데이터셋으로 기존 모델을 추가 학습한
`fire-extinguisher.v1i.yolov8/runs/review_finetune_corrected_labels_20261012/yolov8n/weights/best.pt`
다. 예전 `Guage.v2i` 기본 데이터셋 경로는 제거했다.

SHA-256: `aff903afa7dbd8fea7ef36180141f9b0c6fd7c01513d4c1b8469d27d8956d35f`

이전 10/11 모델 SHA-256은
`0f7bcaa5f1cd098bf81c9b71b8446af037dc5d26747f5401eff772ea63795edb`다.
기존 학습 캐시에는 소등 라벨이 0건으로 남아 있었으므로, 라벨을 변경한
뒤에는 `train/labels.cache`·`valid/labels.cache`를 백업하고 재생성해야 한다.
검증 재현 도구는 저장소 루트의 `scripts/audit_detection_dataset.py`다.
재학습은 `scripts/train_detection_model.py`로 하면 모든 split의 라벨 캐시를
먼저 백업하고 재생성한다. `--data`, `--weights`, `--output-dir`를 명시한다.

| ID | 클래스 | 표시색 |
| --- | --- | --- |
| 0 | Abnormal_left | 빨강 |
| 1 | Abnormal_right | 빨강 |
| 2 | Normal | 초록 |
| 3 | Exit_sign_on | 초록 |
| 4 | Exit_sign_off | 빨강 |

비상구 두 클래스의 원본이 각각 1장인 한계는 남아 있다. 증강본을 분할한
검증 수치는 새로운 환경의 성능을 보장하지 않는다. 수집 조건별 장수와
실영상 신뢰도는 실제 촬영 기록으로 별도 확인한다.
최종 모델의 검증 precision은 0.9353으로 전체 95% 기준을 통과하지 못했다.
현재 모델은 진단용이며, 운영용 채택은 추가 실촬영 검증과 PM 판단이 필요하다.

## 다음 Step

- 운영용 InspectFacility 액션·취소·설비 결과 메시지 연동.
- 실제 카메라에서 조명·위치 변화에 대한 정확도 검증.
- 다중 랙과 셔터 위 물체 검사 설계.
