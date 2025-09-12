# Claude Code를 활용한 Unity 프로젝트 GitHub 관리 가이드

> 🤖 **Claude Code 전용 설명서** - Unity 프로젝트에서 GitHub 작업시 Claude Code에게 제공할 컨텍스트

## 📋 Claude Code에게 전달할 프로젝트 정보

Claude Code와 작업할 때 다음 정보를 미리 제공하세요:

### 🎮 프로젝트 개요
```
이 프로젝트는 Unity AR4 로봇 시뮬레이션 프로젝트입니다.
- Unity 버전: 2022.3 LTS
- 주요 기능: ROS2 통신, AR4 로봇 3D 시뮬레이션, 궤적 시각화
- 플랫폼: Windows (WSL2의 ROS2와 TCP 통신)
```

### 🏗️ 특별한 Unity 파일 구조
```
Unity 프로젝트 특성:
✅ 포함해야 할 폴더: Assets/, ProjectSettings/, Packages/
✅ 포함해야 할 파일: *.meta (에셋 ID 정보)
❌ 제외해야 할 폴더: Library/, Temp/, UserSettings/, Logs/
❌ 제외해야 할 파일: *.tmp (임시 파일)
```

## 🔧 Claude Code 작업 가이드

### 1️⃣ **Git 초기 설정 요청시**

Claude Code에게 다음과 같이 요청하세요:

```
"Unity 프로젝트를 위한 Git 저장소를 설정해주세요. 
다음 사항들을 고려해주세요:

1. Unity 전용 .gitignore 파일 생성 (Library/, Temp/ 제외, .meta 포함)
2. Git LFS 설정 (.gitattributes로 이미지, 오디오, 3D 모델 관리)
3. Unity YAML merge 설정
4. 첫 커밋 생성
5. GitHub 저장소 연결

프로젝트명: unity-ar4-sim
저장소 URL: https://github.com/username/unity-ar4-sim.git"
```

### 2️⃣ **파일 수정/추가 작업시**

Unity 특수 파일들에 대한 안내:

```
"Unity 프로젝트에서 다음 파일들을 수정해주세요:

⚠️ 주의사항:
- .meta 파일은 자동 생성되므로 직접 수정하지 마세요
- Library/ 폴더의 파일들은 건드리지 마세요
- Scene 파일(.unity)은 merge conflict 위험이 있으니 신중하게 처리해주세요

✅ 안전하게 수정 가능한 파일들:
- Assets/Scripts/*.cs (C# 스크립트)
- Assets/Materials/*.mat (머티리얼)
- Assets/Prefabs/*.prefab (프리팹)
- ProjectSettings/*.asset (프로젝트 설정)"
```

### 3️⃣ **커밋 및 푸시 작업시**

```
"Unity 프로젝트 변경사항을 커밋해주세요.

체크리스트:
1. Library/, Temp/ 폴더가 staging에 포함되지 않았는지 확인
2. .meta 파일들이 적절히 포함되었는지 확인
3. 대용량 에셋 파일들이 Git LFS로 처리되는지 확인
4. 의미있는 커밋 메시지 작성 (예: 'Add robot animation controller')
5. 원격 저장소로 푸시"
```

### 4️⃣ **브랜치 작업시**

```
"Unity 프로젝트에서 새 기능 브랜치를 만들어 작업해주세요.

Unity 특화 고려사항:
- Scene 파일은 가급적 한 명이 작업 (merge conflict 방지)
- 대용량 에셋 추가시 Git LFS 확인
- 패키지 변경시 Packages/manifest.json 확인
- 프로젝트 설정 변경시 ProjectSettings/ 폴더 포함"
```

## 📁 중요한 Unity 파일 타입 설명

Claude Code가 알아야 할 Unity 특수 파일들:

### 🎯 **반드시 버전 관리해야 할 파일들**
```bash
Assets/                     # 모든 게임 에셋
├── Scripts/               # C# 스크립트 파일들
├── Scenes/                # Unity Scene 파일들 (.unity)
├── Prefabs/               # 프리팹 파일들 (.prefab)
├── Materials/             # 머티리얼 파일들 (.mat)
└── StreamingAssets/       # 빌드시 포함될 에셋들

ProjectSettings/           # 프로젝트 설정 파일들
├── ProjectSettings.asset  # 메인 프로젝트 설정
├── InputManager.asset     # 입력 설정
└── Physics2DSettings.asset # 물리 설정

Packages/
└── manifest.json          # 패키지 의존성 정보
```

### ❌ **절대 커밋하면 안 되는 폴더들**
```bash
Library/                   # Unity 캐시 폴더 (재생성 가능)
Temp/                     # 임시 파일 폴더
UserSettings/             # 사용자별 에디터 설정
Logs/                     # 로그 파일들
obj/                      # 빌드 임시 파일들
Build/                    # 빌드 결과물
```

## 🤖 Claude Code 명령어 예시

### Git 상태 확인
```bash
# Unity 프로젝트 상태 확인
git status

# 주의: Library/ 나 Temp/ 가 있으면 .gitignore 문제
```

### 안전한 파일 추가
```bash
# 스크립트 파일만 추가
git add Assets/Scripts/

# 특정 에셋만 추가  
git add Assets/Prefabs/RobotController.prefab
git add Assets/Prefabs/RobotController.prefab.meta

# ⚠️ 절대 이렇게 하지 마세요
# git add Library/  # 용량 폭증!
```

### Unity 빌드 결과물 제외
```bash
# 빌드 폴더 확인 후 .gitignore 업데이트
echo "Build/" >> .gitignore
echo "Builds/" >> .gitignore
```

## 🔍 문제 해결 가이드

Claude Code가 마주칠 수 있는 Unity 관련 문제들:

### 🚨 **문제 1: Repository 용량이 너무 클 때**
```
원인: Library/ 폴더가 커밋됨
해결: .gitignore 수정 후 git rm --cached -r Library/
```

### 🚨 **문제 2: .meta 파일이 누락됨**
```
원인: Unity 에디터 설정 문제
해결: Edit → Project Settings → Editor에서
- Version Control Mode: Visible Meta Files
- Asset Serialization Mode: Force Text
```

### 🚨 **문제 3: 대용량 파일 push 실패**
```
원인: Git LFS 미설정
해결: .gitattributes 파일에 LFS 규칙 추가
```

### 🚨 **문제 4: Scene merge conflict**
```
원인: 여러 명이 동시에 Scene 수정
해결: Scene은 가급적 한 명이 작업, 충돌시 Unity Smart Merge 사용
```

## 💡 Claude Code 작업 팁

### ✅ **DO - 이렇게 해주세요**
```bash
# 1. 상태 확인 후 작업
git status
git log --oneline -5

# 2. 단계별 커밋
git add Assets/Scripts/NewFeature.cs
git add Assets/Scripts/NewFeature.cs.meta
git commit -m "Add new robot feature script"

# 3. 의미있는 커밋 메시지
git commit -m "Update robot animation controller

- Add smooth transition between states
- Fix rotation interpolation bug
- Improve performance by 15%"
```

### ❌ **DON'T - 이렇게 하지 마세요**
```bash
# 1. 전체 폴더를 무작정 추가
git add .  # Library/ 포함될 위험

# 2. .meta 파일 무시
git add *.cs  # .meta 파일 누락

# 3. 무의미한 커밋 메시지  
git commit -m "update"  # 무엇을 업데이트했는지 불분명
```

## 📞 Claude Code와의 소통 템플릿

Unity 프로젝트 작업시 다음 템플릿을 사용하세요:

### 🎯 **작업 시작시**
```
"Unity AR4 로봇 시뮬레이션 프로젝트에서 작업합니다.

프로젝트 정보:
- Unity 2022.3 LTS
- ROS2 통신 기반 로봇 제어
- TCP/IP를 통한 WSL2 연동

요청사항: [구체적인 작업 내용]

주의사항: Unity 특수 파일 구조 고려해주세요."
```

### 🔧 **문제 발생시**
```
"Unity 프로젝트에서 Git 문제가 발생했습니다.

현재 상황: [문제 설명]
에러 메시지: [에러 내용]
파일 경로: [관련 파일들]

Unity 프로젝트 특성을 고려해서 해결해주세요."
```

---

> 🤖 **이 가이드를 Claude Code에게 제공하면 Unity 프로젝트에 최적화된 GitHub 관리를 받을 수 있습니다!**