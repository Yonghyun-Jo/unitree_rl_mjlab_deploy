# 🔴 생성 파일. 손으로 고치지 않는다.  python3 deploy/scripts/gen_mode_table_header.py --write
#   학습 사실: mjlab_g1_motion/mode_spec.py @ 45dc2c9c9be0
#   배포 사실: deploy/robots/g1/config/modes.yaml
"""모드 표의 파이썬 판 — GUI(masked_gui.py)가 버튼을 이 표에서 만든다(번호를 코드에 박지 않는다)."""
MODES = [  # (id, name, key, safety)
    (1, "loco", "1", "upright_only"),
    (2, "upper", "2", "upright_only"),
    (3, "track", "3", "upright_only"),
    (4, "playback", "4", "ground_capable"),
    (5, "ground", "5", "ground_capable"),
    (6, "crawl", "6", "ground_capable"),
]
