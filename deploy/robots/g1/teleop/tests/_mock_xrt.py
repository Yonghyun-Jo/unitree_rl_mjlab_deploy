"""_mock_xrt.py — XrtReceiver 단위 테스트용 가짜 xrobotoolkit_sdk (실 하드웨어 없이)."""


class MockXrt:
    def __init__(self):
        self._bts = 1000            # body timestamp(ns) — 테스트에서 수동 증가
        self.body_ok = True
        self.closed = False
        self.X = self.Y = self.A = self.B = False
        self.left_menu = self.right_menu = False

    # body
    def is_body_data_available(self):
        return self.body_ok

    def get_body_joints_pose(self):
        # 24관절 × 7 raw [x,y,z,qx,qy,qz,qw] (pos는 관절 인덱스로 구분, quat은 unit(identity)
        # — zero-norm quat은 GMR retarget()에서 크래시하므로 유효한 quat이어야 함).
        return [[float(i), float(i), float(i), 0.0, 0.0, 0.0, 1.0] for i in range(24)]

    def get_body_timestamp_ns(self):
        return self._bts

    # headset / controllers
    def get_headset_pose(self):
        return [0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0]

    def get_left_controller_pose(self):
        return [0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def get_right_controller_pose(self):
        return [-0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def get_left_trigger(self):
        return 0.0

    def get_right_trigger(self):
        return 0.0

    def get_left_grip(self):
        return 0.0

    def get_right_grip(self):
        return 0.0

    def get_left_axis(self):
        return [0.2, 0.5]

    def get_right_axis(self):
        return [0.3, 0.0]

    def get_left_axis_click(self):
        return False

    def get_right_axis_click(self):
        return False

    def get_left_menu_button(self):
        return self.left_menu

    def get_right_menu_button(self):
        return self.right_menu

    def get_X_button(self):
        return self.X

    def get_Y_button(self):
        return self.Y

    def get_A_button(self):
        return self.A

    def get_B_button(self):
        return self.B

    def close(self):
        self.closed = True
