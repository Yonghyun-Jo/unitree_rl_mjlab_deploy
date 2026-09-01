# 온보드 bootstrap — 두 번째 G1 에서도 «clone 2개 + 명령 2개» 로 같은 상태를 만든다

- 날짜: 2026-09-01
- repo: `unitree_rl_mjlab` / branch: `smooth_mode_switch` (+ `piene_g1_logger` main)
- 상태: 설계 승인(대화) → 구현 계획(writing-plans)으로 이관
- 검토 원장: Obsidian `5. AI_workspace/unitree_rl_mjlab/designs/260901_onboard_piene_ws_portability.md`

---

## 1. 목적

center_g1 온보드 `~/dyros_ws/piene_ws` 에서 지금까지 한 것(제어기 · VR teleop 브릿지 · 로거)을
**다른 G1 에서 똑같이** 세울 수 있게 한다. 방식은 rsync 복제가 아니라 **clone + 재생성**이다.

성공 기준 (전부 기계가 판정한다):
1. `bootstrap.sh` 종료 코드 0 → g1_ctrl Release 빌드 + `robot.sh verify` 통과
2. 브릿지 `--transport udp --arm-estop` 기동 로그에 `hard E-stop: ARMED`, GMR `G("xrobot","unitree_g1")` 스모크 통과
3. `piene_g1_logger/install.sh` 종료 코드 0 → 서비스 active, venv **단독** `import unitree_sdk2py` 성공 (다른 사람 워크스페이스 의존 0)
4. 온보드 총 디스크 **< 2 GB** (현재 7.7 GB), 인터넷 있을 때 **≤ 15 분**
5. center_g1 도 같은 스크립트로 재현되어 5.1 GB `.venv-teleop` 을 대체한다

## 2. 확인된 사실 (설계 근거 — 전부 2026-09-01 실측)

| 사실 | 출처 |
|---|---|
| `unitree_rl_mjlab_deploy` 7.6 G 중 `.venv-teleop` 5.1 G + `.gmr` 1.5 G = 6.6 G (87 %) 가 gitignore 재생성물 | 로봇 `du` |
| `.venv-teleop` 5.1 G 중 `nvidia` 2.9 G + `torch` 0.9 G + `triton` 0.65 G = **4.5 G 가 CUDA 13 torch** — Jetson 은 CUDA 11.4, `torch.cuda.is_available()==False`. 한 바이트도 안 쓰인다 | 로봇 `pip show torch` = `2.13.0+cu130` |
| torch 는 GMR 이 `smplx` 경유로 import 한다(`rot_utils.py`·`kinematics_model.py`). CPU 로 충분 | `.gmr` grep |
| 공식 CPU wheel 존재: `torch-2.13.0+cpu-cp310-cp310-manylinux_2_28_aarch64.whl` (glibc 2.28 ≤ Jetson 2.31) | `download.pytorch.org/whl/cpu` |
| `.gmr/assets` 1.2 G 중 필요한 것은 `unitree_g1` 53 M. GMR 은 LFS 를 쓰지 않는다 → sparse checkout 가능. g1 메시 64 개는 우리 repo STL 사본 | 로봇 `du`, `.gitattributes` 없음 |
| GMR 현재 커밋 `bb1bbe4` (2026-04-02) — 이 커밋에서 실기 검증됨 | `git -C .gmr log -1` |
| `.venv-teleop` 경로를 6 개 문서·스크립트가 참조 → **위치는 바꾸지 않는다** | grep |
| 로거 venv 는 시스템 3.8. `unitree_sdk2py` 는 systemd drop-in `PYTHONPATH=…/garry_ws/unitree_sdk2_python` + `~/.local` egg-link 로만 잡힌다. `cyclonedds 0.10.2` 는 `~/cyclonedds/install/lib/libddsc.so`(공장 경로)에 링크 | 로봇 `env -u PYTHONPATH … import` 실패 |
| 로봇 유닛 파일이 repo 템플릿과 경로가 다르다(손 편집 drift) | `diff` |
| 온보드는 JetPack 5.1.1 / glibc 2.31 → xrt/PC-Service(`--transport local`) 로드 불가. 되는 길 = 노트북 publisher → 온보드 bridge `--transport udp` | 고아 `RUNBOOK_onboard_pico_teleop.md` |
| `g1_logs` 완결 CSV 는 `g1_autorecover.sh --remove-source-files` 로 회수 시 삭제된다. 쌓이는 건 `.csv.partial` 뿐 | 스크립트 + 실측 73→9 |

## 3. 구성요소

전부 🅐 g1 구역(`deploy/robots/g1/**`, `deploy/onboard/**`) · `rules/` · `piene_g1_logger` 다. 🅑 공용 base(`deploy/include/**`)는 건드리지 않는다.

### 3-1. `deploy/onboard/bootstrap.sh` (신규)
멱등. 이미 된 단계는 건너뛴다. 어느 단계든 실패하면 **비0 종료 + 단계 이름 한 줄**. 재실행하면 이어서 한다.

```
PIENE_WS = repo 의 부모 디렉터리 (자동; --ws 로 덮어쓸 수 있음)
① 사전점검   uname -m == aarch64 · glibc 버전 판독(≥2.34 이면 "xrt local 가능" 표시, 아니면 "udp 만") ·
             apt deps(cmake build-essential libboost-all-dev libyaml-cpp-dev zlib1g-dev libfmt-dev libeigen3-dev) 존재 확인.
             🔴 sudo 는 실행하지 않는다 — 없는 패키지는 apt 명령을 출력하고 중단 (공용 기계 규칙)
             github 접근 확인 (git ls-remote, timeout 8)
② .deps      deploy/scripts/build_deps.sh (이미 있으면 skip)
③ g1_ctrl    cmake -DCMAKE_BUILD_TYPE=Release + make -j4 (Release 를 명시 — 로봇 -O3 / com1 -O0 사고 재발 방지)
④ python     uv 없으면 설치(~/.local/bin — 위치 고정 예외로 문서화). `UV_PYTHON_INSTALL_DIR=$PIENE_WS/.uv-python` 로 3.10 설치 → piene_ws 밖 발자국 최소화
⑤ teleop     TELEOP_PY=<uv 3.10> bash deploy/robots/g1/teleop/setup_teleop.sh   (§3-2)
⑥ CLAUDE     deploy/onboard/CLAUDE.robot.md → $PIENE_WS/CLAUDE.md  (이미 있으면 diff 만 보여주고 덮어쓰지 않음)
⑦ 자기검증   g1_ctrl 실행파일 존재 · .venv-teleop python 이 3.10 · import general_motion_retargeting, torch ·
             torch.version.cuda is None · G("xrobot","unitree_g1") 스모크 · du(.venv-teleop+.gmr) < 1.5 G · 요약 표 출력
```
`--only <단계>` / `--skip-teleop` 옵션. setcap 은 `robot.sh deploy` 가 이미 하므로 여기서는 안내만.

### 3-2. `deploy/robots/g1/teleop/setup_teleop.sh` (수정 — 같은 파일, 두 벌 만들지 않음)
| 변경 | 내용 |
|---|---|
| 인터프리터 | `TELEOP_PY` > `uv` 관리 3.10 > `python3`(≥3.10 검사). 3.10 미만이면 중단 |
| GMR | `GMR_COMMIT=bb1bbe4` 기본 핀. `git clone --depth 1 --filter=blob:none --sparse` 후 `git sparse-checkout set general_motion_retargeting assets/unitree_g1 scripts third_party` (+ setup.py 등 루트 파일은 sparse 기본 포함). 기존 full clone 이 있으면 그대로 쓰되 커밋을 확인 |
| torch | **GMR 설치 전에** `pip install "torch==2.13.0" --index-url https://download.pytorch.org/whl/cpu` (x86 도 동일 — 브릿지는 CUDA 를 쓰지 않는다). 그 뒤 `pip install -e .gmr` 는 torch 가 이미 있으므로 CUDA 판을 받지 않는다 |
| xrt | 기존 `XRT_SRC` 로직 유지. glibc < 2.34 이면 시도하지 않고 «이 머신은 udp/zmq 만» 로그 |
| 자기검증 | 기존 스모크 + `torch.version.cuda is None` + 크기 보고 |
| 문서 | 파일 머리 주석의 «LAPTOP side only, 온보드는 필요 없다» 문장을 현재 토폴로지(온보드 bridge udp)로 고친다 |

### 3-3. `deploy/onboard/CLAUDE.robot.md` (신규)
현재 로봇 `piene_ws/CLAUDE.md` 내용을 그대로 이관하고 머리에 두 줄: «원본은 repo `deploy/onboard/CLAUDE.robot.md`, bootstrap ⑥ 이 복사한다 / 로봇에서 고치지 말고 com1 에서 고쳐 push». 로봇 `unitree_rl_mjlab_deploy/CLAUDE.md`(untracked, «git 으로 추적된다」고 잘못 적힘)는 삭제 대상 — 루트 `CLAUDE.md` 는 com1 에서 Obsidian 심링크이므로 **repo 에 추적하지 않는다**.

### 3-4. 고아 파일 구출 (로봇 → repo, 이후 로봇 사본 삭제)
| 로봇 파일 | 갈 곳 | 조치 |
|---|---|---|
| `RUNBOOK_onboard_pico_teleop.md` | `rules/RUNBOOK_onboard_pico_teleop.md` | 머리에 «JetPack 6(glibc≥2.34) 로봇에서만 유효 · JetPack 5 는 `RUNBOOK_laptop_pico_teleop.md` 의 udp 경로» 추가. `/opt/apps/roboticsservice` 가 실제로 설치돼 있음을 «되돌리기» 절에 기록 |
| `pico_sniff.py` | `deploy/robots/g1/teleop/tools/pico_sniff.py` | 그대로. README «점검용」 에 한 줄 |
| `vr_relay_send.py` · `vr_relay_recv.py` | `deploy/robots/g1/teleop/legacy/` + `legacy/README.md` | «Linux PC 에서 xrt local + 하드 E-stop → 로봇에 /dev/shm 중계. 현재는 udp+`--arm-estop` 가 대체. 22.04 Linux PC 가 로봇 옆에 생기면 되살린다» |
| `robot_state_logger.py` · `onboard_teleop_setup.sh` · `sim2sim_setup_x86.sh` | Obsidian `unitree_rl_mjlab/legacy/` 사본만 | repo 에 넣지 않음 (대체물: piene_g1_logger · bootstrap.sh · RUNBOOK_g1_from_scratch Phase 4) |
| `RUNBOOK_real_robot_mode1.md` · `SYSTEM_OVERVIEW.md` (7월 사본) · `.vscode/` · `.xrt-pybind/` | — | 로봇에서 삭제 (repo `rules/` 가 최신; xrt 는 이 머신에서 불가) |
`deploy/robots/g1/teleop/README.md` §C 머리에 ⛔ 한 줄(JetPack 5 불가 → RUNBOOK 링크).

### 3-5. `piene_g1_logger` (별도 repo, main)
| 파일 | 변경 |
|---|---|
| `install.sh` | (a) `REPO=$(pwd)` 로 **유닛을 체크아웃 경로로 생성** — `systemd/g1-logger.service.in` 템플릿의 `__REPO__` 치환 → `~/.config/systemd/user/g1-logger.service`. 기존 `g1-logger.service.d/` drop-in 은 우리 유닛의 것이므로 제거(내용을 로그에 남기고) (b) `.sdk/unitree_sdk2_python` 을 공식 repo(`unitreerobotics/unitree_sdk2_python`)에서 `--depth 1` clone (c) `CYCLONEDDS_HOME=${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}` 로 `pip install cyclonedds==0.10.2` → `pip install -e .sdk/unitree_sdk2_python` (d) 끝에 **venv 단독** `python -c "import unitree_sdk2py, cyclonedds"` + `pytest -q` — 실패하면 비0 종료하고 `systemctl enable --now` 를 하지 않는다 |
| `systemd/g1-logger.service` → `g1-logger.service.in` | `%h/piene_g1_logger` 하드코딩 제거, `__REPO__` 플레이스홀더 |
| `pyproject.toml` | `requires-python = ">=3.8"` (온보드 3.8 이 실제) |
| `.gitignore` | `.sdk/` |
| `README.md`/`DEPLOY.md` | 1.3 «SDK 를 따로 설치」 절을 install.sh 자동화로 갱신. `CYCLONEDDS_HOME` 이 다른 로봇에선 다를 수 있음을 적음 |

center_g1 이관: `robot.sh sync-code piene_g1_logger` → `bash install.sh` (서비스 ~1 분 중단; 실기 세션 밖에서).

### 3-6. com1 측 문서
`claude_rules/procedures/robot_bridge.md §5-2` «새 로봇 붙일 때» 를 새 순서로 교체:
`robot.sh check` → `register` → `ssh -A -o ControlPath=none g1 git clone --depth 1 -b smooth_mode_switch …` ×2 → `bootstrap.sh` → `install.sh` → `robot.sh verify` → 첫날 하드웨어 기록.
`ws_root` 필드 · `robot.sh bootstrap` 서브커맨드는 **이번 범위 밖**(새 로봇 홈 규약 확인 뒤).

## 4. 인터페이스 · 데이터 흐름

**변화 없음.** `/dev/shm/g1_vr_ref`(VrRef 276 B)·`/dev/shm/g1_estop` 계약, UDP :5556, systemd 유닛 이름 `g1-logger`, WS :8700, `.venv-teleop` 경로, `run_g1_with_gui.sh`, `robot.sh deploy/verify` 전부 그대로.

## 5. 에러 처리

- 단계 실패 = 즉시 중단 + 단계 이름. 부분 성공 상태는 재실행으로 이어진다(멱등).
- sudo · 인터넷이 필요한 항목은 ① 에서 미리 판정해 **명령만 출력**한다. 스크립트가 sudo 를 부르지 않는다.
- 자기검증에 실패하면 «설치됐다»고 출력하지 않는다 (2026-08-13 로거 일주일 사망 재발 방지).
- 로봇 쪽 파일 삭제(§3-4)는 스크립트가 아니라 사람이 `robot.sh sh` 로 한다 — bootstrap 은 만들기만 한다.

## 6. 테스트 계획

1. 정적: `bash -n` + `shellcheck` (bootstrap.sh · setup_teleop.sh · install.sh)
2. `setup_teleop.sh` 를 **com1 x86** 에서 `REPO_ROOT` 임시 복사본으로 실행 → import · `torch.version.cuda is None` · 크기 < 1.5 G 확인 (aarch64 wheel 은 로봇에서만 확인 가능 — 여기선 로직만)
3. `install.sh` 를 com1 에서 `--dry-run`(유닛 생성·템플릿 치환·SDK clone 까지, systemctl 은 출력만) 으로 확인
4. center_g1(실기 세션 밖): `.venv-teleop` → `.venv-teleop.bak_260901` 로 옮기고 `bootstrap.sh` 실행 → ⑦ 자기검증 + `robot.sh verify` + 브릿지 `--transport udp --arm-estop` 기동 로그. 통과하면 `.bak` 삭제(5.1 G 회수)
5. logger: `install.sh` 후 `systemctl --user is-active` · 60 초 뒤 `g1_logs` 에 새 CSV · `pytest -q` · com1 `robot.sh pull` 로 회수 확인
6. 모든 로봇 작업은 «com1 수정 → commit → push → 로봇 pull» 경로. 로봇에서 편집하지 않는다.

## 7. 하지 않는 것
별도 teleop repo · 오프라인 wheel 번들 · `.venv-teleop` 위치 이동 · 🅑 공용 base 수정 · `robot.sh` 신규 서브커맨드 · `config.robots.yaml: ws_root` · 로봇에서 직접 편집.

## 8. 남은 결정 (기본값으로 진행, 사용자가 뒤집으면 따른다)
- `vr_relay_*` = legacy 보존 (폐기 아님)
- center_g1 `g1_logs` 의 stale `.partial` 27 개(1.2 G) = `.csv` 로 rename 해 회수(삭제 아님). 이 설계 범위 밖, 별도 한 줄 작업
- 새 G1 이 JetPack 6 이면 `--transport local` 경로가 살아난다 → 첫날 `glibc` 판독 결과로 갈린다 (bootstrap ① 이 표시)
