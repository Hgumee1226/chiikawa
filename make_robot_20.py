
from pathlib import Path
import xml.etree.ElementTree as ET

BASE_DIR = Path(__file__).resolve().parent

SOURCE = BASE_DIR / "robot_50.urdf"
OUTPUT = BASE_DIR / "robot_low_20.urdf"

# 실험 조건
WHEEL_RADIUS = 0.025
CHASSIS_HALF_HEIGHT = 0.020
TARGET_CLEARANCE = 0.020

WHEEL_JOINT_NAMES = {
    "front_left_joint",
    "front_right_joint",
    "rear_left_joint",
    "rear_right_joint",
}

if not SOURCE.exists():
    raise FileNotFoundError(
        f"원본 URDF를 찾을 수 없습니다: {SOURCE}"
    )

if OUTPUT.exists():
    raise FileExistsError(
        f"이미 {OUTPUT.name} 파일이 존재합니다. "
        "기존 실험 파일을 보호하기 위해 덮어쓰지 않습니다."
    )

tree = ET.parse(SOURCE)
root = tree.getroot()

# 지상고 = 바퀴 반지름 - 차체 반높이 - 바퀴축 상대 Z
target_joint_z = (
    WHEEL_RADIUS
    - CHASSIS_HALF_HEIGHT
    - TARGET_CLEARANCE
)

modified = 0

for joint in root.findall("joint"):
    if joint.get("name") not in WHEEL_JOINT_NAMES:
        continue

    origin = joint.find("origin")

    if origin is None:
        raise ValueError(
            f"{joint.get('name')}: origin이 없습니다."
        )

    xyz = [
        float(value)
        for value in origin.get("xyz").split()
    ]

    xyz[2] = target_joint_z

    origin.set(
        "xyz",
        " ".join(f"{value:.6f}" for value in xyz)
    )

    modified += 1

if modified != 4:
    raise ValueError(
        f"바퀴 조인트가 4개여야 합니다. 현재 {modified}개"
    )

tree.write(
    OUTPUT,
    encoding="utf-8",
    xml_declaration=True
)

print("==============================")
print("20 mm URDF CREATED")
print("==============================")
print(f"Source          : {SOURCE.name}")
print(f"Output          : {OUTPUT.name}")
print(f"Target clearance: {TARGET_CLEARANCE * 1000:.1f} mm")
print(f"Wheel joint Z   : {target_joint_z:.4f} m")
print(f"Modified joints : {modified}")
print("==============================")
