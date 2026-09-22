# 🔴 생성 파일. 손으로 고치지 않는다.  deploy/scripts/gen_mode5_presets_header.py --write
#   학습 사실: mjlab_g1_motion/mode5_presets.py @ 4ea947cbe0c7
"""mode5 자세 버튼 표의 파이썬 판 — GUI 가 버튼을 이 표에서 만든다. shm 의 m5_preset = index + 1."""
ENTER_PRESET = 0
PRESETS = [  # (index, name, key, n)
    (0, "직립", "z", 145),
    (1, "베어 크롤(배아래)", "", 9),
    (2, "크랩(배위)", "h", 11),
    (3, "네발 기기", "x", 17),
    (4, "낮은 포복", "c", 7),
    (5, "바닥 앉기", "b", 8),
    (6, "드러누움", "n", 24),
    (7, "한쪽 낮춘 지지(L)", "", 5),
    (8, "한쪽 낮춘 지지(R)", "", 5),
]
