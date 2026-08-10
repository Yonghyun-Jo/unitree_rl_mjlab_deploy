"""
run_g1_teleop_networked.py  (com1 에서 실행)

Unitree G1 양팔 placo 텔레옵을, XR 입력을 '노트북 ZMQ 스트림'에서 받아 구동.

동작:
  1) 리타게팅 컨트롤러가 내부에서 만드는 XrClient 를 NetworkXrClient 로 몽키패치
  2) 공식 teleop_unitree_g1_placo.py 와 동일한 설정으로 실행
     단, 우리는 모션 트래커가 없으므로 config 에서 motion_tracker 를 제거
     (없으면 팔꿈치 위치 태스크가 초기위치에 고정되어 IK 를 방해할 수 있음)

전제:
  - 이 repo(XRoboToolkit-Teleop-Sample-Python) 의 conda env 안에서 실행
  - network_xr_client.py 가 같은 폴더에 있어야 함
  - 노트북에서 pico_publisher.py 가 com1 으로 발행 중 (PICO 앱 Send ON)

실행:
  conda activate <teleop_env>
  python run_g1_teleop_networked.py                # 기본 스케일 1.0
  python run_g1_teleop_networked.py --scale-factor 0.5
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# --- xrobotoolkit_sdk stub 주입 (컨트롤러 import 전에!) ---
# xr_client.py 는 import 시점에 `import xrobotoolkit_sdk` 를 실행한다.
# com1 에는 PICO/PC-Service 가 없어 이 SDK 가 없지만, 우리는 실제로 호출하지 않고
# NetworkXrClient 로 XrClient 를 대체하므로 import 만 통과하면 된다(빈 stub 이면 충분).
sys.modules.setdefault("xrobotoolkit_sdk", types.ModuleType("xrobotoolkit_sdk"))

# --- XrClient 몽키패치 (컨트롤러 생성 전에!) ---
import network_xr_client  # noqa: E402
import xrobotoolkit_teleop.common.base_teleop_controller as _base  # noqa: E402
import xrobotoolkit_teleop.common.xr_client as _xrc  # noqa: E402

_base.XrClient = network_xr_client.NetworkXrClient
_xrc.XrClient = network_xr_client.NetworkXrClient
# --------------------------------------------------

import tyro  # noqa: E402
from xrobotoolkit_teleop.simulation.placo_teleop_controller import (  # noqa: E402
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH  # noqa: E402


def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "unitree/g1/g1_dual_arm.urdf"),
    scale_factor: float = 1.0,
):
    """노트북 XR 스트림으로 G1 양팔 placo 텔레옵 구동."""
    # 트래커 없음 → motion_tracker 제거, 손목 pose 로만 팔 IK
    config = {
        "left_arm": {
            "link_name": "left_rubber_hand",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
        },
        "right_arm": {
            "link_name": "right_rubber_hand",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
        },
    }

    controller = PlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
    )

    # 팔이 자연스러운 자세를 유지하도록 관절 정규화(soft)
    joints_task = controller.solver.add_joints_task()
    default_joints = {
        "left_shoulder_pitch_joint": 0.3,
        "left_shoulder_roll_joint": 0.2,
        "left_shoulder_yaw_joint": 0.0,
        "left_elbow_joint": 1.0,
        "left_wrist_roll_joint": 0.0,
        "left_wrist_pitch_joint": 0.0,
        "left_wrist_yaw_joint": 0.0,
        "right_shoulder_pitch_joint": 0.3,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_yaw_joint": 0.0,
        "right_elbow_joint": 1.0,
        "right_wrist_roll_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
        "right_wrist_yaw_joint": 0.0,
    }
    joints_task.set_joints(default_joints)
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("Unitree G1 양팔 텔레옵 시작 (입력: 노트북 ZMQ 스트림)")
    print("  - 왼쪽 컨트롤러 -> 왼팔 (left_rubber_hand)")
    print("  - 오른쪽 컨트롤러 -> 오른팔 (right_rubber_hand)")
    print("  - grip 을 누르는 동안만 팔이 컨트롤러를 추종")
    print("  - meshcat 뷰어 URL 이 아래에 출력됨 (헤드리스면 Tailscale 로 포워딩해 열기)")

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
