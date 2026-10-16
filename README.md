# OnPlant Hardware

<p align="center">
  <strong>OnPlant Raspberry Pi 로봇의 센서·구동·음성·디스플레이 런타임</strong><br/>
  Raspberry Pi에서 독립 서비스로 실행되는 로봇 하드웨어 제어 코드입니다.
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi-C51A4A?logo=raspberrypi&logoColor=white" alt="Raspberry Pi"/>
  <img src="https://img.shields.io/badge/Runtime-systemd-000000?logo=linux&logoColor=white" alt="systemd"/>
  <img src="https://img.shields.io/badge/Status-Developing-orange" alt="Developing"/>
</p>

## 프로젝트 소개

OnPlant Hardware는 반려식물 관리 로봇의 Raspberry Pi 측 코드를 모아 둔 저장소입니다. 센서 수집, 이동 제어, 카메라 전송, 음성 인터랙션, 전면 디스플레이를 각각 독립된 사용자 서비스로 실행해 한 기능의 오류가 다른 기능을 불필요하게 중단시키지 않도록 구성합니다.

| 항목 | 내용 |
|---|---|
| 프로젝트 | OnPlant 반려식물 로봇 |
| 담당 영역 | Raspberry Pi 하드웨어·런타임 |
| 주요 기술 | Python, Raspberry Pi, I2C/GPIO, LiDAR, systemd |
| 서버 연동 | OnPlant Server REST API |

## 동작 구조

```text
systemd 사용자 서비스
├─ 센서 수집 ──▶ 로컬 최신값 캐시 ──▶ OnPlant Server
├─ 전면 디스플레이
├─ 음성 인식·응답 ──▶ 로컬 명령 큐
├─ 주행 명령 처리 ──▶ LiDAR 기반 주행 프로세스
└─ USB 카메라 영상 전송 ──▶ OnPlant Server
```

이동 제어는 `robot_main.py`가 명령을 받아 주행 프로세스를 시작·중지하고, 실제 주행 루프는 `lidar_fsm_drive.py`에서 처리합니다. 상세한 설치 및 운영 절차는 [하드웨어 런타임 가이드](hardware/README.md)를 참고하세요.

## 주요 구성

| 구성 | 역할 |
|---|---|
| Sensor | I2C/GPIO 센서 측정 및 서버 전송 |
| Drive | 로봇 명령 처리, 모터 제어, LiDAR 기반 주행 |
| Voice | 웨이크 워드·음성 인식 및 음성 응답 |
| Display | Raspberry Pi 로컬 상태 화면 제공 |
| Camera | USB 카메라 프레임 전송 |
| Diagnostics | 센서·모터·LiDAR 점검용 수동 도구 |

## 기술 구성

- **Runtime:** Python, Linux, systemd user services
- **Hardware I/O:** I2C, GPIO, Serial, 모터 드라이버
- **Sensors:** LiDAR, 조도·온습도·토양 센서
- **Communication:** HTTP API, 로컬 파일 캐시 및 명령 큐
- **Voice / Display:** 로컬 음성 처리와 웹 기반 전면 화면

## 설치 및 실행

Raspberry Pi에서 저장소를 준비한 뒤 가상환경과 의존성을 설치합니다.

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
pip install -r requirements-local-voice.txt
```

서비스 설치, Vosk 모델 준비, 환경 변수 설정 및 시작 절차는 [하드웨어 런타임 가이드](hardware/README.md)에 정리되어 있습니다. `hardware/onplant-robot.env.example`을 기준으로 환경을 구성하고, 실제 API 키와 장치 설정은 기기에서 별도로 입력하세요.

## 프로젝트 구조

```text
.
├─ robot_main.py                 # 로봇 명령 및 주행 프로세스 관리
├─ lidar_fsm_drive.py            # LiDAR 기반 주행 상태 처리
├─ sensor_sender.py              # 센서 수집·전송
├─ voice_assistant_local.py      # 로컬 음성 인터랙션
├─ usb_camera_sender.py          # 카메라 데이터 전송
├─ local_runtime.py              # 로컬 명령 및 런타임 상태
├─ movement/                     # 이동 제어 모듈
├─ hardware/                     # 장치 드라이버·디스플레이·systemd 서비스
└─ tools/diagnostics/            # 수동 점검 도구
```

## 운영 시 주의사항

- 실제 장치에서 실행하기 전 환경 변수의 서버 주소, 로봇 ID, API 키와 센서 핀 설정을 확인합니다.
- 모터 또는 LiDAR 진단 도구는 해당 장치를 점유 중인 주행 서비스를 중지한 뒤 실행합니다.
- 장시간 무인 실행이 필요하면 Raspberry Pi 사용자 서비스의 로그인·linger 설정을 확인합니다.

---

OnPlant 프로젝트의 하드웨어 런타임 영역을 기록하고 공유하기 위한 저장소입니다.
