# Unity AR4 Simulation

Unity 시뮬레이션 프로젝트로 AR4 로봇 시각화 및 제어를 담당합니다.

## 🎯 주요 기능

- **3D AR4 로봇 모델** - 실시간 관절 움직임 시뮬레이션 (6-DOF ArticulationBody 기반)
- **ROS2 통신** - TCP/IP를 통한 WSL2 ROS2 연동 (30Hz 실시간 데이터 교환)
- **궤적 시각화** - MoveIt 경로를 노란색 waypoint로 표시
- **실시간 제어** - 키보드, UI 슬라이더 및 ROS2 명령어를 통한 관절 제어
- **자세 관리** - 홈 포즈 설정 및 저장된 자세 관리 기능

## 🛠️ 시스템 요구사항

### 필수 요구사항
- **Unity**: 6000.2.3f1 (Unity 2023.3 LTS 이상 권장)
- **OS**: Windows 10/11 (WSL2 필수)
- **RAM**: 최소 8GB (16GB 권장)
- **GPU**: DirectX 11 지원 GPU
- **네트워크**: WSL2와 TCP 통신 (포트 10000)

### Unity 패키지 의존성
- **ROS TCP Connector**: GitHub 최신 버전 (자동 설치됨)
- **URDF Importer**: v0.5.2 (자동 설치됨)
- **Universal Render Pipeline**: 17.2.0
- **Input System**: 1.14.2
- **UI Toolkit**: Unity 기본 포함

## 🚀 설치 및 실행

### 1. 전제 조건 확인

#### WSL2 및 ROS2 설정
```bash
# WSL2에서 실행
# 1. ROS2 워크스페이스 빌드 (필요시)
cd ~/ros2-ar4-ws
colcon build

# 2. 환경 설정
source ~/ros2-ar4-ws/install/setup.bash

# 3. ROS TCP Endpoint 실행 (Unity 연결용)
# A) WSL2/호스트에서 직접 실행
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# B) Docker 컨테이너로 실행 (권장)
#   - 메타 저장소(ar4-stack) 루트에서 다음 실행
#     docker compose build && docker compose up

# 4. WSL2 IP 주소 확인 (Unity에서 설정할 IP)
hostname -I
```

### 2. Unity 프로젝트 설정

#### 프로젝트 열기
```
Unity Hub → Add → 이 프로젝트 폴더 선택 → Unity 6000.2.3f1로 열기
```

#### 패키지 자동 설치 확인
- Unity 에디터가 처음 열릴 때 Package Manager에서 ROS 패키지들이 자동으로 다운로드됩니다
- 진행 상황은 Unity 하단 진행률 표시줄에서 확인 가능
- 완료까지 약 2-3분 소요

#### ROS 연결 설정
1. **Hierarchy**에서 `ROS2Manager` GameObject 찾기
2. **Inspector**에서 `ROS2Manager` 컴포넌트의 설정값 확인:
   - **ROS IP Address**: WSL2 IP로 변경 (예: `172.27.145.180`)
   - **ROS Port**: `10000` (기본값)
   - **Publish Rate**: `30` (Hz, 기본값)

### 3. 씬 구성 확인
- **메인 씬**: `Assets/Scenes/SampleScene.unity` 열기
- **로봇 모델**: Hierarchy에서 `ar4_mk3` GameObject 확인
- **UI**: Canvas 하위의 AR4 제어 패널 확인

### 4. 실행
```
Play 버튼 클릭 → Game 뷰에서 ROS 연결 상태 확인 (좌상단 녹색/빨간색 표시)
```

## 🔧 주요 컴포넌트

### 핵심 스크립트 (`Assets/Robots/AR4/Scripts/`)
- **`ROS2Manager.cs`** - ROS TCP 통신 관리 및 토픽 발행/구독
  - WSL2 IP/포트 설정
  - 관절 상태 30Hz 발행 (`/joint_states`)
  - 관절 명령 구독 (`/joint_command`)
- **`AR4JoggerPanel.cs`** - UI 기반 로봇 관절 제어
  - 6개 관절 개별 제어 슬라이더
  - 홈 포즈 설정 및 자세 저장/로딩
  - ROS2와 연동된 관절 각도 제어
- **`TrajectoryVisualizer.cs`** - MoveIt 궤적 경로 시각화
- **`MultiJointJogger.cs`** - 키보드 기반 다중 관절 제어
- **`SimpleJointJogger.cs`** - 개별 관절 제어 유틸리티

### 로봇 모델 (`Assets/Robots/AR4/meshes/`)
- **ar4_mk3/** - AR4 로봇 본체 링크들 (base_link ~ link_6)
- **ar4_gripper/** - AR4 그리퍼 구성 요소
- 각 링크는 **ArticulationBody** 기반 물리 시뮬레이션

### 중요한 Prefabs
- **`ROSConnectionPrefab.prefab`** - ROS TCP Connector 설정 (Assets/Resources/)
- **로봇 링크 Prefabs** - 각 관절별 3D 모델과 물리 속성

### Scene 구성
- **`SampleScene.unity`** - 메인 시뮬레이션 씬
  - AR4 로봇 모델 (완전 조립된 상태)
  - ROS2Manager GameObject
  - UI Canvas (관절 제어 패널)
  - 카메라 및 조명 설정

## 📡 ROS2 토픽 및 메시지

### Unity에서 발행 (Publish)
- **`/joint_states`** (`sensor_msgs/JointState`)
  - 주기: 30Hz (설정 가능)
  - 6개 관절의 현재 각도 (라디안)
  - 속도 및 토크 정보 (현재는 0으로 설정)
  - frame_id: "ar4"

### Unity에서 구독 (Subscribe)
- **`/joint_command`** (`sensor_msgs/JointState`)
  - ROS2에서 Unity로 관절 제어 명령 전송
  - 6개 관절의 목표 각도 (라디안)
  - 실시간으로 Unity 로봇 관절에 반영

### 메시지 형식 예시
```yaml
# /joint_states 발행 데이터
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "ar4"
name: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
position: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]  # 라디안
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # 현재 미사용
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]        # 현재 미사용
```

## 🔍 문제 해결

### 🔴 Unity 프로젝트 관련

#### "Package Manager Error" 또는 ROS 패키지 다운로드 실패
```bash
# 해결 방법
1. Window → Package Manager → In Project → ROS TCP Connector 확인
2. 인터넷 연결 상태 확인
3. Unity Hub에서 프로젝트 재시작
4. 필요시 패키지 수동 설치:
   - Package Manager → + → Add package from git URL
   - https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

#### "Missing Script" 에러
```bash
# 해결 방법
1. Project 창에서 Assets/Robots/AR4/Scripts/ 폴더 확인
2. 모든 .cs 파일이 존재하는지 확인
3. Console에서 컴파일 에러 메시지 확인 후 수정
```

### 🔴 ROS2 연결 관련

#### "ROS2: Disconnected" (빨간색 표시)
```bash
# 1단계: WSL2 상태 확인
wsl --status
wsl --list --verbose

# 2단계: WSL2에서 ROS TCP Endpoint 실행 확인
# 새 WSL2 터미널에서:
source ~/ros2-ar4-ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# 3단계: 방화벽 확인 (Windows)
# Windows Defender 방화벽에서 포트 10000 인바운드 허용 규칙 추가
```

#### WSL2 IP 주소 자주 변경되는 문제
```bash
# WSL2 IP 고정 방법 (선택사항)
# Windows PowerShell (관리자 권한):
wsl -d Ubuntu -u root ip addr add 192.168.50.2/24 broadcast 192.168.50.255 dev eth0 label eth0:1

# Unity에서 고정 IP 사용: 192.168.50.2
```

#### 토픽 데이터가 전송되지 않는 문제
```bash
# ROS2에서 토픽 확인
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /joint_command

# Unity Console에서 에러 메시지 확인
# ROS2Manager의 publishRate 설정 확인 (기본값: 30Hz)
```

### 🔴 로봇 제어 관련

#### 로봇이 움직이지 않음
```bash
# 확인 사항
1. Game 뷰에서 ROS 연결 상태 (녹색이어야 함)
2. Hierarchy에서 ROS2Manager GameObject 활성화 확인
3. AR4JoggerPanel의 joints 배열에 6개 관절 모두 할당되었는지 확인
4. Console에서 "Setting joint X to Y degrees" 메시지 확인
```

#### 관절이 이상하게 움직임
```bash
# 해결 방법
1. 각 ArticulationBody의 Joint Limits 확인
2. ROS2Manager에서 라디안↔도 변환 확인
3. Unity Physics 설정에서 Fixed Timestep 확인 (0.02 권장)
```

#### UI 슬라이더가 작동하지 않음
```bash
# 확인 사항
1. Canvas의 UI Toolkit 설정 확인
2. AR4JoggerPanel에서 UI Document 할당 확인
3. Console에서 UI 관련 에러 메시지 확인
```

### 🔴 성능 관련

#### Unity 실행 시 프레임 드롭
```bash
# 최적화 방법
1. Quality Settings → Ultra → Good으로 변경
2. URP Asset에서 Shadow Distance 감소
3. Camera Render Distance 조정
4. ROS publishRate를 30Hz → 10Hz로 감소
```

### 🔴 일반적인 디버깅 방법

#### 로그 확인
```bash
# Unity Console 로그
ROS2Manager: 연결 상태 및 토픽 메시지
AR4JoggerPanel: 관절 제어 로그

# WSL2 ROS2 로그
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 --ros-args --log-level DEBUG
```

#### 네트워크 연결 테스트
```bash
# Windows에서 WSL2로 ping 테스트
ping 172.27.145.180  # WSL2 IP

# WSL2에서 포트 리스닝 확인
netstat -tlnp | grep 10000
```

## 🤝 ROS2 워크스페이스 연동

이 Unity 프로젝트는 [ros2-ar4-ws](https://github.com/kwangchae/ros2-ar4-ws)와 함께 동작합니다.

### 필수 ROS2 패키지
```bash
# WSL2 Ubuntu에서 실행
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-tcp-endpoint

# ROS2 워크스페이스 클론 및 빌드
cd ~
git clone https://github.com/kwangchae/ros2-ar4-ws.git
cd ros2-ar4-ws
colcon build
```

### 실행 순서
```bash
# 1. ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2-ar4-ws/install/setup.bash

# 2. ROS TCP Endpoint 실행 (Unity 연결용)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# 3. (선택) MoveIt 실행 (별도 터미널)
ros2 launch ar4_moveit_config demo.launch.py

# 4. Unity 프로젝트 실행 (Windows)
```

### 통합 테스트
```bash
# ROS2에서 Unity로 관절 명령 전송 테스트
ros2 topic pub /joint_command sensor_msgs/JointState '{
  name: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  position: [0.5, -0.5, 0.8, -0.3, 1.2, 0.0]
}'

# Unity에서 ROS2로 관절 상태 수신 확인
ros2 topic echo /joint_states
```

## 📝 개발 노트 및 기술 세부사항

### Unity 버전 및 설정
- **Unity**: 6000.2.3f1 (2024.3 기반)
- **Render Pipeline**: Universal Render Pipeline (URP) 17.2.0
- **Physics**: ArticulationBody 기반 로봇 관절 시뮬레이션
- **좌표계**: Unity 좌표계 (Y-up, 왼손 좌표계)

### ROS2 통신 세부사항
- **ROS TCP Connector**: GitHub 최신 버전 사용
- **메시지 형식**: ROS2 Humble과 호환
- **좌표 변환**:
  - Unity (도) ↔ ROS2 (라디안)
  - Unity 왼손 좌표계 ↔ ROS2 오른손 좌표계
- **통신 주기**: 30Hz (조정 가능)

### 로봇 모델 구성
- **관절 수**: 6-DOF (6개 회전 관절)
- **물리 엔진**: Unity ArticulationBody (실제 로봇과 유사한 동작)
- **관절 제한**: 각 관절별 min/max 각도 제한 적용
- **그리퍼**: AR4 그리퍼 모델 포함 (제어는 별도 구현 필요)

### 성능 최적화
```csharp
// ROS2Manager.cs에서 성능 관련 설정
public float publishRate = 30f;  // 30Hz로 제한하여 성능 확보
public bool clampToLimits = true; // 관절 한계값 적용으로 안정성 확보
```

### 확장 가능성
- **궤적 시각화**: TrajectoryVisualizer.cs로 MoveIt 경로 표시
- **멀티 로봇**: 여러 AR4 로봇 동시 시뮬레이션 가능
- **센서 추가**: 카메라, 라이다 등 센서 시뮬레이션 확장 가능
- **그리퍼 제어**: 현재 미구현, 추가 개발 필요

## 🤖 Claude Code 사용자를 위한 안내

이 Unity 프로젝트에서 Claude Code와 작업할 때는 **[CLAUDE_CODE_GUIDE.md](./CLAUDE_CODE_GUIDE.md)**를 참고하세요.

### 빠른 참고사항:
- **포함**: Assets/, ProjectSettings/, *.meta
- **제외**: Library/, Temp/, UserSettings/
- **Git LFS**: 이미지, 오디오, 3D 모델
- **Unity 설정**: Visible Meta Files + Force Text

## 🚀 빠른 시작 체크리스트

새로운 개발자가 이 프로젝트를 빠르게 시작할 수 있도록 준비된 체크리스트입니다:

### ✅ 전제 조건
- [ ] Windows 10/11 설치됨
- [ ] WSL2 설치 및 Ubuntu 배포판 구성됨
- [ ] Unity Hub 및 Unity 6000.2.3f1 설치됨
- [ ] Git 설치됨

### ✅ ROS2 환경 설정
- [ ] WSL2에서 ROS2 Humble 설치됨
- [ ] ros2-ar4-ws 클론 및 빌드 완료
- [ ] ROS TCP Endpoint 패키지 설치됨
- [ ] WSL2 IP 주소 확인됨

### ✅ Unity 프로젝트 설정
- [ ] 프로젝트를 Unity Hub에 추가
- [ ] Unity 에디터에서 프로젝트 열기
- [ ] Package Manager에서 ROS 패키지 자동 다운로드 대기
- [ ] SampleScene.unity 열기
- [ ] ROS2Manager의 IP 주소 설정

### ✅ 연결 테스트
- [ ] WSL2에서 ROS TCP Endpoint 실행
- [ ] Unity Play 모드 실행
- [ ] Game 뷰에서 "ROS2: Connected" 녹색 표시 확인
- [ ] ROS2에서 `/joint_states` 토픽 수신 확인

### 🔧 문제 발생 시
문제가 발생하면 위의 **🔍 문제 해결** 섹션을 참고하거나 [Issues](https://github.com/kwangchae/unity-ar4-sim/issues)에 문의하세요.

---

**관련 저장소**
- [ros2-ar4-ws](https://github.com/kwangchae/ros2-ar4-ws) - ROS2 워크스페이스
- [ar4-stack](https://github.com/kwangchae/ar4-stack) - 통합 메타 저장소
