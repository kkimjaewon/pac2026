# pac_ai_inference

A(김나현) | AI 추론 계층. 패키지 이름은 ai_inference.

## 실행

    ros2 launch ai_inference ai_inference.launch.py

기본값은 detector_mode=auto, primary_impl=yolo(모델 로드에 실패하면 V로 동작), color_v_min=170 입니다. 색+형상 P로 쓰려면 primary_impl:=color 입니다. YOLO에는 onnxruntime이 필요합니다.
170은 팀 월드(franka_world.sdf) 조명 기준이고, 월드 조명이 바뀌면 다시 맞춰야 합니다.

    ros2 launch ai_inference ai_inference.launch.py color_v_min:=200
## onnxruntime 설치 (YOLO P 검출기)

기본 P 검출기가 YOLO라서 onnxruntime이 필요합니다. 설치되어 있지 않으면 YOLO 모델을 불러오지 못하고 V로만 동작합니다.

ros2 launch는 시스템 Python(/usr/bin/python3)으로 노드를 실행하므로, 같은 Python에 설치합니다(Ubuntu 24.04, Python 3.12).

    python3 -m pip install --user --break-system-packages onnxruntime

설치 확인:

    python3 -c "import onnxruntime; print(onnxruntime.__version__)"

1.30.0과 1.31.0에서 동작을 확인했습니다. A를 켠 뒤 A.log에 "YOLO P 검출기 로드"가 나오면 정상입니다.
## 입출력

- 입력: /depth_camera/image_raw, /depth_camera/depth/image_raw, /depth_camera/camera_info, TF map -> depth_camera_link
- 출력: /ai/control_command (intelion_interfaces/msg/ControlCommand), seq는 1부터 시작
- 로그: ~/pac_logs/ai_*.csv

## 장애 주입 (실행 중)

    ros2 param set /ai_inference_node fault_mode delay      # none | delay | stop | bad_value
    ros2 param set /ai_inference_node det_fault_mode p_miss # none | p_delay | p_bad_value | p_miss

## 가짜 A (B 개발용, 패키지 ai_mock)

    ros2 run ai_mock fake_ai_node

### 10/08 A–B–C 좌표·시간 계약 보완

A는 map/m 단위의 철판 표면 좌표만 보내고 작업 높이(+0.10m)는 B가 한 번 적용한다. camera_link의 x축과 optical z축을 구분하며 카메라 배치는 TF에서 읽는다. 유효 결과라도 200ms 초과 또는 음수 age, NaN/Inf 좌표는 신뢰도 0으로 무효화한다(현재 로컬 보완).

`capture_mono_ns`는 현재 SensorAdapter의 동기화된 RGB/깊이 수신 callback 시작 시각이다. 실제 센서 촬영 시각이나 카메라→A 전송 지연을 포함하는 값이 아니다. `publish_mono_ns = capture_mono_ns + age_ns`는 CommandPublisher의 발행 직전 age 측정 경계이며 DDS 실제 송출/수신 완료 시각이 아니다. A 내부 구간만 같은 시계로 차감하고 다른 PC의 B/C 절대 monotonic 값과 빼지 않는다.

CSV 헤더는 현재 `boot_id`(메시지 writer_boot_id와 동일), `seq`, `published`다. stop 행은 published=0, seq와 publish_mono_ns가 비어 있으므로 A/B/C 연결에서 제외한다. A 로그 파일은 `<log_dir>/ai_<날짜_시각>.csv`이며 이름을 ai.csv로 가정하지 말고 실제 경로를 사용한다. 재시작은 boot+seq로 구분하며 최신 B의 used_ai_boot_id와 used_ai_seq가 실제 사용 출처다.
