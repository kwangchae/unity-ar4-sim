# Unity AR4-MK3 Simulator

Unity 기반 AR4-MK3 로봇 시뮬레이터입니다. ROS2와의 실시간 연동을 위한 3D 물리 시뮬레이션을 제공합니다.

## 🎯 주요 기능

- **3D 로봇 시뮬레이션**: AR4-MK3 물리 기반 관절 제어
- **실시간 ROS2 연동**: TCP 통신을 통한 양방향 데이터 동기화
- **수동 제어 UI**: Jogger Panel을 통한 직관적 관절 조작
- **궤적 시각화**: 3D 경로 표시 및 waypoint 마커
- **자동화 시퀀스**: Pick & Place 등 복잡한 작업 시뮬레이션

## 🛠️ 시스템 요구사항

- **Unity**: 2022.3 LTS 이상
- **OS**: Windows 10/11
- **패키지**: 
  - URDF Importer v0.5.2
  - ROS-TCP-Connector

## 📁 프로젝트 구조

```
Assets/
└── Robots/
    └── AR4/
        ├── ar4_mk3.unity.urdf      # 로봇 모델 정의
        ├── meshes/                 # 3D 메시 파일
        ├── Scripts/                # C# 제어 스크립트
        ├── UI/                     # 사용자 인터페이스
        └── ROS2/                   # ROS2 통신 스크립트
```

## 🔧 주요 컴포넌트

### Unity Scripts
- **ROS2Manager.cs**: ROS2 TCP 통신 관리
- **AR4JoggerPanel.cs**: 수동 관절 제어 UI
- **TrajectoryVisualizer.cs**: 3D 궤적 시각화

### ROS2 Integration Scripts  
- **joint_sequence_controller.py**: 자동 관절 시퀀스
- **moveit_unity_bridge.py**: MoveIt 경로 계획 연동
- **simple_moveit_test.py**: 기본 연결 테스트

## 🚀 설치 및 실행

### 1. Unity 패키지 설치
```
Window > Package Manager > + > Add package from git URL:
- https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2
- https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

### 2. ROS2 연동 설정
- `Robotics > ROS Settings`에서 ROS IP 주소를 WSL2 IP로 설정
- Default Port: `10000`

### 3. 씬 설정
1. `ar4_mk3.unity` 로봇 프리팹을 씬에 배치
2. AR4 GameObject에 `ROS2Manager` 컴포넌트 추가
3. UI Controller에 `AR4JoggerPanel` 컴포넌트 추가
4. (선택) `TrajectoryVisualizer` 추가

## 📊 ROS2 토픽

| 토픽 | 방향 | 메시지 타입 | 설명 |
|------|------|------------|------|
| `/joint_states` | Unity → ROS2 | `sensor_msgs/JointState` | 현재 관절 상태 |
| `/joint_command` | ROS2 → Unity | `sensor_msgs/JointState` | 관절 명령 |
| `/trajectory_preview` | ROS2 → Unity | `sensor_msgs/JointState` | 궤적 미리보기 |

## 🎮 사용법

### 수동 제어
1. Play 모드 실행
2. 화면 좌측의 Jogger Panel 사용
3. 슬라이더로 각 관절 각도 조정
4. [홈 포즈], [저장/불러오기] 기능 활용

### ROS2 연동 테스트
1. ROS2 브릿지 연결 후 Console에서 "Connected" 확인
2. WSL2에서 관절 명령 전송:
```bash
ros2 topic pub --once /joint_command sensor_msgs/JointState '{
  name: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  position: [0.3, -0.2, 0.5, 0.0, 0.4, 0.0]
}'
```

## 🔍 디버깅

### 연결 문제
- Unity Console에서 ROS2Manager 로그 확인
- Game View 좌상단 연결 상태 확인 (녹색 = 연결됨)
- WSL2 IP 주소 변경 시 ROS Settings 업데이트

### 관절 움직임 문제  
- ArticulationBody 컴포넌트의 Joint Type 확인
- Joint limits와 드라이브 설정 검증
- Unity 창이 포커스된 상태에서 테스트

## 📝 개발 노트

- Unity Physics timestep: 50Hz (0.02s)
- ROS2 publish rate: 30Hz
- Coordinate system: Unity 좌표계 (Y-up) ↔ ROS 좌표계 변환
- Joint angles: Unity degrees ↔ ROS radians 변환

## 📄 라이선스

MIT License

---
**개발환경**: Unity 2022.3 LTS, Windows 11  
**연동 시스템**: ROS2 Jazzy (WSL2 Ubuntu 24.04)