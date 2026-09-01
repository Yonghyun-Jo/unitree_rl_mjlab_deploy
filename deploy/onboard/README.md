# deploy/onboard — 새 G1 온보드에 «같은 상태» 를 만드는 곳

로봇은 공용 기계다. 내 것은 `~/dyros_ws/piene_ws/` 안에만 만든다 (규칙 원문: `claude_rules/projects/unitree_rl_mjlab.md`).

## 새 로봇 첫날 (com1 에서, 노트북이 로봇 옆에 있을 때)
```bash
cd ~/piene_automation
./robot_bridge/robot.sh check                       # 3단 🟢
./robot_bridge/robot.sh register <이름> "<위치>"      # 지문으로 정체 고정 (IP 는 두 로봇이 같다)
# clone 2개 — private repo 라 com1 키를 «빌려서» (ControlPath=none 필수: 마스터 연결은 -A 없이 열려 있다)
ssh -A -o ControlPath=none g1 'mkdir -p ~/dyros_ws/piene_ws && cd ~/dyros_ws/piene_ws \
  && git clone --depth 1 -b smooth_mode_switch git@github.com:Yonghyun-Jo/unitree_rl_mjlab_deploy.git \
  && git clone --depth 1 git@github.com:Yonghyun-Jo/piene_g1_logger.git'
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && bash deploy/onboard/bootstrap.sh --check'
#   apt 가 모자라면 사람이 로봇에서 sudo apt install … 후 다시
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/unitree_rl_mjlab_deploy && nohup bash deploy/onboard/bootstrap.sh > /tmp/bootstrap.log 2>&1 &'
#   10~15 분. tail: robot.sh sh 'tail -5 /tmp/bootstrap.log'
./robot_bridge/robot.sh sh 'cd ~/dyros_ws/piene_ws/piene_g1_logger && bash install.sh'
./robot_bridge/robot.sh verify                      # 게이트 (setcap 은 robot.sh deploy 가 하거나 사람이)
```
첫날 기록할 것: `bootstrap ①` 의 glibc 판정(xrt local 가능 여부) · 손/머리 장착물/판 rev(memory `ask-hardware-config-before-sim2real-diagnosis`).

## 파일
- `bootstrap.sh` — 7단계 멱등. sudo 안 부름.
- `CLAUDE.robot.md` — 로봇 `piene_ws/CLAUDE.md` 의 원본. 로봇에서 고치지 말고 여기서 고쳐 push.
- 정책 슬롯 무게는 git 이 아니라 `deploy/scripts/policy_slot.py push` (rsync).
