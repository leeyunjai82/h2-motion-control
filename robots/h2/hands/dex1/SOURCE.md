# Dex1-1 URDF · 메시 출처

- 저장소: https://github.com/unitreerobotics/unitree_ros — `robots/dexterous_hand_description/dex1_1/`
- 커밋: 3ae893d905d11795ddf1075825ee5010364a1109 (2026-10-30)
- 라이선스: BSD 3-Clause (같은 폴더 `LICENSE`, Unitree Robotics)
- 가져온 것: `dex1_1.urdf` (원본 그대로), `meshes/Dex1_base_link.STL`, `Dex1_finger_link_1.STL`, `Dex1_finger_link_2.STL`.
  `Camera.STL` 은 이 로봇에서 안 쓰므로 안 가져옴 (URDF 의 Camera 링크는 3D 보기에서 건너뜀)

쓰는 곳: `gripper/vr_teleop.py` 3D 보기 (표시만). H2 손목에 붙이는 위치 · 방향은 `gripper.yaml` `vr.dex1_view`.
