<!-- 원본: unitree_rl_mjlab_deploy/deploy/onboard/CLAUDE.robot.md — bootstrap.sh ⑥ 이 piene_ws/CLAUDE.md 로 복사한다.
     로봇에서 이 파일을 고치지 않는다. com1 에서 고쳐 push → 로봇 pull → bootstrap --only 6. -->
# piene_ws — 실로봇 온보드 (Jetson)

**이 파일은 이 머신에만 있다.** git 저장소 바깥이라 push되지 않고 다른 머신으로 따라가지 않는다.
여기에는 이 로봇 컴퓨터에서만 참인 규칙을 적는다.

공통 규칙은 repo 의 `rules/`(`SYSTEM_OVERVIEW.md`·`RUNBOOK_*.md`) 와 com1 의 `claude_rules/projects/unitree_rl_mjlab.md` 에 있다. 충돌하면 이 파일이 이긴다.

## 이 머신

- Unitree 로봇 온보드 컴퓨터. Jetson (aarch64, tegra R35). SSH로 접속해 작업 중이다.
- `eth0` `192.168.123.164` — 로봇 내부망. 모터와 센서가 이 망에 붙어 있다.
- 워크스페이스 루트: `/home/unitree/dyros_ws/piene_ws`

## 작업 경계

- `/home/unitree/dyros_ws/piene_ws` **안쪽은 자유롭게 수정**한다. `unitree_rl_mjlab_deploy/`도 포함이다.
- 이 경로 **바깥은 수정하지 않는다.** 특히 `~/dyros_ws/` 아래 다른 사람의 워크스페이스(예: `garry_ws`·`sanghyuk_ws`·`kw_ws`), 홈 디렉토리
  설정 파일, 시스템 경로, 전역 site-packages. 읽기와 참조는 자유롭다.
- 바깥 경로에서 `git` 쓰기 명령이나 포맷터·린터 자동 수정을 실행하지 않는다.
- 실수로 바깥에 쓰기를 시도했다면 즉시 멈추고 알린다.

## 안전 — 실제 모터가 붙어 있다

**빌드는 허용, 실행은 제한.**

- 확인 없이 해도 되는 것: `cmake` / `make` / `deploy/scripts/build_deps.sh` 빌드, 컴파일,
  테스트, 정적 분석, 로그 조회, 로봇 상태 **구독(읽기)**.
- **실행 전 반드시 사용자 확인을 받는 것:**
  - `deploy/robots/*/build/` 아래 배포 바이너리
  - 모터 활성화, 정책 실행, 상태 전환 (damping → stand → policy)
  - `deploy/robots/g1/teleop/` teleop 브리지, `vr_replay.py`
  - `192.168.123.x`로 나가는 DDS/SDK 명령 토픽 발행
- 실행을 제안할 때는 "실행할까요?"로 끝내지 말고 조건을 짚는다:
  로봇이 행잉되어 있는가, 주변에 사람이 있는가, E-stop이 손에 있는가.
- `sudo`, 패키지 설치, 네트워크·서비스 설정 변경은 하지 않는다. 온보드 복구는 비싸다.

## 실행 전 빌드 정합성 — 실기체를 움직이기 전 반드시 확인

**가장 흔한 사고 원인: pull/브랜치 전환/소스 수정 후 재빌드를 잊고 옛 바이너리를 그대로 돌리는 것.**

왜 위험한가: `config.yaml` · `deploy.yaml` · `policy.onnx` 는 **런타임에 로드**된다. 그러나
`src/` · `include/`(State_Mimic, MaskedLocoController 등) · `main.cpp` 의 **컴파일된 코드는 옛날 것**
그대로다. 그래서 "옛 바이너리 + 방금 갈아끼운 새 정책/클립" 조합이 만들어지고, 정책이 기대하는 obs 구성과
바이너리가 만드는 obs가 어긋나 **조용히 쓰레기 액션 → 발산**한다. (실측: 2026-08-13, 07-14 빌드 바이너리가
당일 pull한 cwc_scratch 정책을 물고 mode4에서 발산.)

**로봇을 움직이는 배포 바이너리(`deploy/robots/*/build/`)를 실행 제안하기 전에 매번:**

1. `git -C unitree_rl_mjlab_deploy log -1 --format=%cd` (마지막 커밋 시각) 과
   `git status`(수정된 소스) 를 본다. 방금 `pull`/`checkout`/소스 수정을 했다면 → **무조건 재빌드부터.**
2. 바이너리 mtime 이 아래 전부보다 최신인지 확인한다:
   `src/` · `include/` · `main.cpp` · `config/config.yaml` · 활성 `policy_dir`의 `deploy.yaml`·`exported/policy.onnx`.
   하나라도 바이너리보다 새로우면 → **stale. 재빌드 없이는 실행 제안 금지.**
3. 재빌드는 확인 없이 해도 되는 작업이다. 오히려 **재빌드를 건너뛴 실행 제안이 규칙 위반**이다.

"방금 pull 했다"는 말이 나오면, 실행 얘기를 꺼내기 전에 재빌드 필요 여부부터 자동으로 점검하고 알린다.

## git

- 저장소는 `unitree_rl_mjlab_deploy/` 와 `piene_g1_logger/` 둘이다. `piene_ws` 자체는 저장소가 아니다.
- 현재 브랜치가 `main`이 아닐 수 있다. 커밋·푸시는 사용자가 요청할 때만 한다.
- `piene_g1_logger/` 의 SDK 는 그 repo 의 `.venv` 안에 있다 — 다른 사람 워크스페이스(`garry_ws` 등)에 기대지 않는다.
