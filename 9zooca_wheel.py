import math
import time
import tempfile
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================
# 기본 설정
# ==========================================
DT = 1 / 480

RADIUS = 0.025
TREAD_THICKNESS = 0.002
TREAD_WIDTH = 0.018
ARC_RADIUS = RADIUS - TREAD_THICKNESS / 2
ARC_SEGMENTS = 24

FRAME_RADIUS = 0.021
FRAME_WIDTH = 0.004
FRAME_THICKNESS = 0.0015

# 최대 전개 관절 회전량: 60도
DEPLOY_ANGLE = math.radians(60)

# 곡선 다리 방향
ARC_DIRECTION = -1

# 일정한 저속으로 전진
FORWARD_SPEED = 2.0       # rad/s
DRIVE_TORQUE = 1.0
BRAKE_TORQUE = 2.0

STEP_HEIGHT = 0.025
STEP_FRONT_X = 0.20

START_POSITION = [-0.40, 0, 0.041]

WHEELS = [
    ("front_left",  0.105,  0.120),
    ("front_right", 0.105, -0.120),
    ("rear_left",  -0.105,  0.120),
    ("rear_right", -0.105, -0.120),
]


def inertial(mass, ixx, iyy, izz):
    return f"""
    <inertial>
      <mass value="{mass}"/>
      <inertia
        ixx="{ixx}" iyy="{iyy}" izz="{izz}"
        ixy="0" ixz="0" iyz="0"/>
    </inertial>
    """


def beam(ax, az, bx, bz, width, thickness,
         material, collision=False):
    dx = bx - ax
    dz = bz - az

    length = math.hypot(dx, dz)
    angle = -math.atan2(dz, dx)

    origin = (
        f'<origin xyz="{(ax + bx) / 2} 0 {(az + bz) / 2}" '
        f'rpy="0 {angle} 0"/>'
    )

    geometry = (
        '<geometry>'
        f'<box size="{length} {width} {thickness}"/>'
        '</geometry>'
    )

    result = (
        f'<visual>{origin}{geometry}'
        f'<material name="{material}"/></visual>'
    )

    if collision:
        result += f'<collision>{origin}{geometry}</collision>'

    return result


def make_ring():
    result = []

    for i in range(48):
        a = 2 * math.pi * i / 48
        b = 2 * math.pi * (i + 1) / 48

        result.append(
            beam(
                FRAME_RADIUS * math.cos(a),
                FRAME_RADIUS * math.sin(a),
                FRAME_RADIUS * math.cos(b),
                FRAME_RADIUS * math.sin(b),
                FRAME_WIDTH,
                FRAME_THICKNESS,
                "gold",
            )
        )

    return "".join(result)


def make_robot_xml(opened):
    deploy_angle = (
        ARC_DIRECTION * DEPLOY_ANGLE if opened else 0.0
    )

    xml = [
        '<?xml version="1.0"?>',
        '<robot name="rigid_transform_robot">',

        '<material name="red">'
        '<color rgba="0.72 0.12 0.10 1"/></material>',

        '<material name="gold">'
        '<color rgba="0.86 0.62 0.25 1"/></material>',

        '<material name="silver">'
        '<color rgba="0.67 0.72 0.76 1"/></material>',

        '<material name="blue">'
        '<color rgba="0.16 0.29 0.43 1"/></material>',

        '<link name="base">',
        inertial(2.8, 0.00971, 0.01496, 0.02392),

        '<visual>'
        '<geometry><box size="0.25 0.20 0.04"/></geometry>'
        '<material name="blue"/>'
        '</visual>',

        '<collision>'
        '<geometry><box size="0.25 0.20 0.04"/></geometry>'
        '</collision>',

        '</link>',
    ]

    for name, x, y in WHEELS:
        frame = f"{name}_frame"

        xml += [
            f'<link name="{frame}">',
            inertial(0.02, 0.000003, 0.000006, 0.000003),

            '<visual>'
            '<origin rpy="1.57079632679 0 0"/>'
            '<geometry>'
            '<cylinder radius="0.0055" length="0.012"/>'
            '</geometry>'
            '<material name="gold"/>'
            '</visual>',

            make_ring(),
        ]

        # 중앙 스포크
        for k in range(6):
            angle = 2 * math.pi * k / 6

            xml.append(
                beam(
                    0, 0,
                    FRAME_RADIUS * math.cos(angle),
                    FRAME_RADIUS * math.sin(angle),
                    0.003,
                    0.0015,
                    "gold",
                )
            )

        # 힌지 지지대와 핀
        for k in range(3):
            angle = 2 * math.pi * k / 3
            px = ARC_RADIUS * math.cos(angle)
            pz = ARC_RADIUS * math.sin(angle)

            xml.append(
                beam(
                    FRAME_RADIUS * math.cos(angle),
                    FRAME_RADIUS * math.sin(angle),
                    px,
                    pz,
                    0.004,
                    0.002,
                    "silver",
                )
            )

            xml.append(
                '<visual>'
                f'<origin xyz="{px} 0 {pz}" '
                'rpy="1.57079632679 0 0"/>'
                '<geometry>'
                '<cylinder radius="0.002" length="0.021"/>'
                '</geometry>'
                '<material name="silver"/>'
                '</visual>'
            )

        xml += [
            '</link>',

            # 바퀴 전체 구동 관절
            f'<joint name="{name}_drive" type="continuous">',
            '<parent link="base"/>',
            f'<child link="{frame}"/>',
            f'<origin xyz="{x} {y} -0.015"/>',
            '<axis xyz="0 1 0"/>',
            '<dynamics damping="0.001" friction="0"/>',
            '</joint>',
        ]

        # 외곽 곡선 다리 3개
        for k in range(3):
            start = 2 * math.pi * k / 3
            px = ARC_RADIUS * math.cos(start)
            pz = ARC_RADIUS * math.sin(start)

            arc = f"{name}_arc_{k}"

            xml += [
                f'<link name="{arc}">',
                inertial(0.01, 0.000004, 0.000006, 0.000004),
            ]

            span = ARC_DIRECTION * math.radians(119)

            for j in range(ARC_SEGMENTS):
                a = start + span * j / ARC_SEGMENTS
                b = start + span * (j + 1) / ARC_SEGMENTS

                xml.append(
                    beam(
                        ARC_RADIUS * math.cos(a) - px,
                        ARC_RADIUS * math.sin(a) - pz,
                        ARC_RADIUS * math.cos(b) - px,
                        ARC_RADIUS * math.sin(b) - pz,
                        TREAD_WIDTH,
                        TREAD_THICKNESS,
                        "red",
                        collision=True,
                    )
                )

            xml += [
                '</link>',

                # 고정 관절이므로 주행 중 더 꺾이지 않음
                f'<joint name="{name}_deploy_{k}" type="fixed">',
                f'<parent link="{frame}"/>',
                f'<child link="{arc}"/>',
                f'<origin xyz="{px} 0 {pz}" '
                f'rpy="0 {deploy_angle} 0"/>',
                '</joint>',
            ]

    xml.append('</robot>')
    return "\n".join(xml)


def get_drive_joints(robot):
    joints = {}

    for joint in range(p.getNumJoints(robot)):
        name = p.getJointInfo(robot, joint)[1].decode()

        if name.endswith("_drive"):
            joints[name] = joint

    return joints


def minimum_collision_height(robot):
    heights = [p.getAABB(robot, -1)[0][2]]

    for joint in range(p.getNumJoints(robot)):
        heights.append(p.getAABB(robot, joint)[0][2])

    return min(heights)


def load_robot(path, position, orientation):
    robot = p.loadURDF(
        str(path),
        basePosition=position,
        baseOrientation=orientation,
        useFixedBase=False,
    )

    for link in range(-1, p.getNumJoints(robot)):
        p.changeDynamics(
            robot,
            link,
            lateralFriction=0.8 if link == -1 else 1.0,
            rollingFriction=0.001,
            spinningFriction=0.001,
            linearDamping=0.04,
            angularDamping=0.04,
        )

    joints = get_drive_joints(robot)

    for joint in joints.values():
        p.setJointMotorControl2(
            robot,
            joint,
            p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=BRAKE_TORQUE,
        )

    return robot, joints


def switch_shape(robot, joints, path):
    """정지 상태에서 형상을 교체하고 바퀴 회전각을 유지."""
    position, orientation = p.getBasePositionAndOrientation(robot)

    phases = {
        name: p.getJointState(robot, joint)[0]
        for name, joint in joints.items()
    }

    old_min_z = minimum_collision_height(robot)

    new_robot, new_joints = load_robot(
        path,
        position,
        orientation,
    )

    for name, joint in new_joints.items():
        p.resetJointState(
            new_robot,
            joint,
            targetValue=phases[name],
            targetVelocity=0,
        )

    # 형상 교체 시 지면 관통을 줄이기 위한 높이 보정
    new_min_z = minimum_collision_height(new_robot)
    lift = max(0.0, old_min_z - new_min_z)

    if lift > 0:
        position = [
            position[0],
            position[1],
            position[2] + lift + 0.0005,
        ]

        p.resetBasePositionAndOrientation(
            new_robot,
            position,
            orientation,
        )

    p.resetBaseVelocity(
        new_robot,
        linearVelocity=[0, 0, 0],
        angularVelocity=[0, 0, 0],
    )

    p.removeBody(robot)

    return new_robot, new_joints


def main():
    if p.connect(p.GUI) < 0:
        raise RuntimeError("PyBullet GUI 연결 실패")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        p.setPhysicsEngineParameter(numSolverIterations=150)

        plane = p.loadURDF("plane.urdf")
        p.changeDynamics(plane, -1, lateralFriction=0.8)

        step_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, STEP_HEIGHT / 2],
        )

        step_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.4, 0.4, STEP_HEIGHT / 2],
            rgbaColor=[0.65, 0.68, 0.72, 1],
        )

        step = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=step_collision,
            baseVisualShapeIndex=step_visual,
            basePosition=[
                STEP_FRONT_X + 0.4,
                0,
                STEP_HEIGHT / 2,
            ],
        )
        p.changeDynamics(step, -1, lateralFriction=0.8)

        # ==================================
        # 자동 제어 설정
        # ==================================
        SENSOR_RANGE = 0.05
        SENSOR_FRONT = 0.132
        SENSOR_LOCAL_Z = -0.030

        # 문턱 모서리보다 충분히 안쪽까지 통과
        CLEARANCE_MARGIN = 0.015
        CONFIRM_TIME = 0.20

        with tempfile.TemporaryDirectory(
            prefix="auto_transform_"
        ) as folder:
            folder = Path(folder)

            closed_path = folder / "closed.urdf"
            opened_path = folder / "opened.urdf"

            closed_path.write_text(
                make_robot_xml(False),
                encoding="utf-8",
            )
            opened_path.write_text(
                make_robot_xml(True),
                encoding="utf-8",
            )

            identity = p.getQuaternionFromEuler([0, 0, 0])

            robot, joints = load_robot(
                closed_path,
                START_POSITION,
                identity,
            )

            opened = False
            driving = True

            # ROUND_APPROACH:
            #   원형 바퀴로 접근
            # OPEN_CLIMB:
            #   전개 상태로 등반
            # ROUND_AFTER:
            #   통과 후 원형 복귀
            state = "ROUND_APPROACH"

            clear_time = 0.0
            debug_count = 0
            sensor_line = -1

            def rear_links_of(body):
                """뒷바퀴 프레임과 곡선 부품 링크 번호."""
                result = {
                    "rear_left": [],
                    "rear_right": [],
                }

                for j in range(p.getNumJoints(body)):
                    link_name = p.getJointInfo(body, j)[12].decode()

                    for prefix in result:
                        if link_name.startswith(prefix + "_"):
                            result[prefix].append(j)

                return result

            rear_links = rear_links_of(robot)

            def detect_obstacle(body):
                """
                차체 앞쪽의 낮은 위치에서 전방 ray 발사.
                현재 실험의 문턱을 감지한다.
                """
                position, orientation = (
                    p.getBasePositionAndOrientation(body)
                )

                start, _ = p.multiplyTransforms(
                    position,
                    orientation,
                    [SENSOR_FRONT, 0, SENSOR_LOCAL_Z],
                    [0, 0, 0, 1],
                )

                end, _ = p.multiplyTransforms(
                    position,
                    orientation,
                    [
                        SENSOR_FRONT + SENSOR_RANGE,
                        0,
                        SENSOR_LOCAL_Z,
                    ],
                    [0, 0, 0, 1],
                )

                hit = p.rayTest(start, end)[0]

                detected = hit[0] == step
                distance = (
                    hit[2] * SENSOR_RANGE
                    if detected else None
                )

                return detected, distance, start, end

            def rear_has_cleared(body, links):
                """
                양쪽 뒷바퀴가 상판에서 지지되고,
                뒷바퀴 전체가 문턱 앞면을 지났는지 확인.
                """
                contacts = p.getContactPoints(
                    bodyA=body,
                    bodyB=step,
                )

                for side_links in links.values():
                    # 상판 법선은 대체로 위쪽을 향한다.
                    supported = any(
                        cp[3] in side_links
                        and cp[9] > 0.1
                        and cp[7][2] > 0.7
                        for cp in contacts
                    )

                    if not supported:
                        return False

                    # 해당 뒷바퀴 모든 링크의 가장 뒤쪽 X
                    minimum_x = min(
                        p.getAABB(body, link)[0][0]
                        for link in side_links
                    )

                    if minimum_x <= (
                        STEP_FRONT_X + CLEARANCE_MARGIN
                    ):
                        return False

                return True

            p.resetDebugVisualizerCamera(
                cameraDistance=0.9,
                cameraYaw=25,
                cameraPitch=-20,
                cameraTargetPosition=[0.05, 0, 0.05],
            )

            print("================================")
            print("자동 장애물 등반 모드")
            print("전방 감지 → 전개 → 등반 → 원형 복귀")
            print(f"목표 각속도: {FORWARD_SPEED:.1f} rad/s")
            print("SPACE : 주행 정지 / 재개")
            print("R     : 처음부터 다시")
            print("Q     : 종료")
            print("================================")

            while p.isConnected():
                keys = p.getKeyboardEvents()

                def pressed(key):
                    code = key if isinstance(key, int) else ord(key)
                    return bool(
                        keys.get(code, 0) & p.KEY_WAS_TRIGGERED
                    )

                if pressed("q"):
                    break

                if pressed(32):
                    driving = not driving
                    print("주행 재개" if driving else "브레이크")

                if pressed("r"):
                    p.removeBody(robot)

                    robot, joints = load_robot(
                        closed_path,
                        START_POSITION,
                        identity,
                    )

                    rear_links = rear_links_of(robot)

                    opened = False
                    driving = True
                    state = "ROUND_APPROACH"
                    clear_time = 0.0

                    print("초기화 — 원형 바퀴로 접근")

                # ==============================
                # 전방 장애물 감지
                # ==============================
                detected, distance, ray_start, ray_end = (
                    detect_obstacle(robot)
                )

                sensor_line = p.addUserDebugLine(
                    ray_start,
                    ray_end,
                    lineColorRGB=(
                        [1, 0.1, 0.1]
                        if detected else [0.1, 0.8, 0.1]
                    ),
                    lineWidth=2,
                    replaceItemUniqueId=sensor_line,
                )

                if driving:
                    if state == "ROUND_APPROACH":
                        if detected:
                            print(
                                f"[감지] 전방 문턱까지 "
                                f"{distance * 1000:.1f} mm"
                            )

                            robot, joints = switch_shape(
                                robot,
                                joints,
                                opened_path,
                            )

                            rear_links = rear_links_of(robot)

                            opened = True
                            state = "OPEN_CLIMB"
                            clear_time = 0.0

                            print("[전개] 전개 바퀴로 등반 시작")

                    elif state == "OPEN_CLIMB":
                        if rear_has_cleared(robot, rear_links):
                            clear_time += DT
                        else:
                            clear_time = 0.0

                        if clear_time >= CONFIRM_TIME:
                            robot, joints = switch_shape(
                                robot,
                                joints,
                                closed_path,
                            )

                            rear_links = rear_links_of(robot)

                            opened = False
                            state = "ROUND_AFTER"
                            clear_time = 0.0

                            print(
                                "[통과] 양쪽 뒷바퀴 통과 확인"
                            )
                            print(
                                "[복귀] 원형 바퀴로 계속 전진"
                            )

                    # ROUND_AFTER에서는 같은 문턱에
                    # 다시 반응하지 않고 계속 전진한다.

                # ==============================
                # 일정한 속도로 바퀴 구동
                # ==============================
                command = FORWARD_SPEED if driving else 0.0
                torque = (
                    DRIVE_TORQUE if driving else BRAKE_TORQUE
                )

                for joint in joints.values():
                    p.setJointMotorControl2(
                        bodyUniqueId=robot,
                        jointIndex=joint,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=command,
                        force=torque,
                    )

                p.stepSimulation()

                debug_count += 1

                if debug_count % 480 == 0:
                    position, _ = (
                        p.getBasePositionAndOrientation(robot)
                    )
                    velocity, _ = p.getBaseVelocity(robot)

                    print(
                        f"[{state}] "
                        f"X={position[0]:.3f} m, "
                        f"Vx={velocity[0]:.3f} m/s, "
                        f"전개={opened}"
                    )

                time.sleep(DT)

    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()