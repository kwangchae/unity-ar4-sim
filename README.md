# Unity AR4 Simulation

Unity 시뮬레이션 프로젝트로 AR4 로봇 시각화 및 제어를 담당합니다.

## 🎯 주요 기능

- **3D AR4 로봇 모델** - 실시간 관절 움직임 시뮬레이션
- **ROS2 통신** - TCP/IP를 통한 WSL2 ROS2 연동  
- **궤적 시각화** - MoveIt 경로를 노란색 waypoint로 표시
- **실시간 제어** - 키보드 및 MoveIt 명령어 반영

## 🛠️ 시스템 요구사항

- **Unity**: 2022.3 LTS 이상
- **OS**: Windows 10/11
- **네트워크**: WSL2와 TCP 통신 (포트 10000)

## 🚀 설치 및 실행

### 1. Unity 프로젝트 열기
```
File → Open Project → 이 폴더 선택
```

### 2. ROS Settings 구성
- Scene에서 `ROSConnectionPrefab` 선택
- Inspector에서 ROS IP를 WSL2 IP로 설정:
```bash
# WSL2에서 IP 확인
hostname -I
```

### 3. 실행
- Play 버튼 클릭
- Console에서 ROS 연결 상태 확인

## 🔧 주요 컴포넌트

### Scripts/
- `ROSUnityConnection.cs` - ROS TCP 통신 관리
- `AR4Controller.cs` - 로봇 관절 제어
- `TrajectoryVisualizer.cs` - 경로 시각화
- `JointStateSubscriber.cs` - 관절 상태 구독

### Prefabs/
- `AR4_Robot.prefab` - 완전한 AR4 로봇 모델
- `TrajectoryPoint.prefab` - 궤적 waypoint 표시

### Materials/
- `AR4_Body.mat` - 로봇 본체 재질
- `TrajectoryPath.mat` - 궤적 경로 재질

## 📡 ROS2 토픽

### 구독 (Subscribe)
- `/joint_command` - Unity 로봇 제어
- `/trajectory_preview` - 궤적 waypoint 데이터

### 발행 (Publish)  
- `/joint_states` - 현재 관절 상태
- `/unity_status` - Unity 연결 상태

## 🔍 문제 해결

### ROS 연결 안 됨
1. WSL2 IP 주소 확인
2. 방화벽 설정 확인
3. ROS TCP Endpoint 실행 여부 확인

### 로봇이 움직이지 않음
1. Console에서 에러 메시지 확인
2. Joint limits 설정 확인
3. ROS2 토픽 데이터 확인

## 🤝 ROS2 워크스페이스 연동

이 Unity 프로젝트는 [ros2-ar4-ws](https://github.com/kwangchae/ros2-ar4-ws)와 함께 동작합니다:

```bash
# ROS2 워크스페이스 실행
source ~/ros2-ar4-ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## 📝 개발 노트

- Unity 2022.3 LTS 권장
- ROS-TCP-Connector 패키지 사용
- Joint 각도는 라디안 단위 사용
- 좌표계: Unity 좌표계 (Y-up) 사용

---

**관련 저장소**
- [ros2-ar4-ws](https://github.com/kwangchae/ros2-ar4-ws) - ROS2 워크스페이스
- [ar4-stack](https://github.com/kwangchae/ar4-stack) - 통합 메타 저장소