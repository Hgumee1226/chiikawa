
import os
import pybullet as p
import pybullet_data
import time
import math

from contact_motion_analysis import ContactMotionAnalyzer
from contact_mechanics_analysis import ContactMechanicsAnalyzer


# ==========================================
# 실험 조건
# ==========================================

STEP_HEIGHT = 0.0233      # 문턱 높이 [m] = 23.4 mm
WHEEL_RADIUS = 0.025      # 바퀴 반지름 [m] = 25 mm

MOTOR_SPEED = 6.0         # 바퀴 목표 회전속도 [rad/s]
MAX_TORQUE = 1.0          # 바퀴 하나당 최대 토크 [N·m]

SIMULATION_TIME = 10.0    # 한 번의 실험 시간 [s]
TIME_STEP = 1 / 240       # 물리 계산 간격 [s]


# ==========================================
# 현재 main.py 파일의 위치
# ==========================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

ROBOT_PATH = os.path.join(
    BASE_DIR,
    "robot_low_20.urdf"
)


# ==========================================
# PyBullet 시작
# ==========================================

physics_client = p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)

p.setTimeStep(
    TIME_STEP
)


# 처음 카메라 위치
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=40,
    cameraPitch=-25,
    cameraTargetPosition=[0.2, 0, 0]
)


# ==========================================
# 바닥 생성
# ==========================================

plane_id = p.loadURDF(
    "plane.urdf"
)

p.changeDynamics(
    plane_id,
    -1,
    lateralFriction=0.80
)


# ==========================================
# 문턱 생성
# ==========================================

STEP_FRONT_X = 0.20

STEP_LENGTH = 1.0
STEP_WIDTH = 0.8


step_collision = p.createCollisionShape(
    p.GEOM_BOX,

    halfExtents=[
        STEP_LENGTH / 2,
        STEP_WIDTH / 2,
        STEP_HEIGHT / 2
    ]
)


step_visual = p.createVisualShape(
    p.GEOM_BOX,

    halfExtents=[
        STEP_LENGTH / 2,
        STEP_WIDTH / 2,
        STEP_HEIGHT / 2
    ]
)


step_id = p.createMultiBody(

    baseMass=0,

    baseCollisionShapeIndex=step_collision,

    baseVisualShapeIndex=step_visual,

    # 문턱 앞면이 x = 0.20 m 위치
    basePosition=[
        STEP_FRONT_X + STEP_LENGTH / 2,
        0,
        STEP_HEIGHT / 2
    ]
)


p.changeDynamics(
    step_id,
    -1,
    lateralFriction=0.8
)


# ==========================================
# 로봇 불러오기
# ==========================================

robot = p.loadURDF(

    ROBOT_PATH,

    basePosition=[
        -0.5,
        0,
        0.040
    ],

    useFixedBase=False
)


# 차체 마찰 / 감쇠
p.changeDynamics(

    robot,
    -1,

    lateralFriction=0.80,

    linearDamping=0.04,

    angularDamping=0.04
)


# ==========================================
# Joint 확인
# ==========================================

number_of_joints = p.getNumJoints(
    robot
)


print("\n==============================")
print("Robot joint information")
print("==============================")


for i in range(number_of_joints):

    info = p.getJointInfo(
        robot,
        i
    )

    joint_name = info[1].decode(
        "utf-8"
    )

    print(
        i,
        joint_name
    )


    # 바퀴 마찰 설정
    p.changeDynamics(

        robot,
        i,

        lateralFriction=1.0,

        rollingFriction=0.001,

        spinningFriction=0.001
    )


# 현재 URDF에서는
# 0, 1, 2, 3번 Joint가 네 바퀴
wheel_joints = [
    0,
    1,
    2,
    3
]


# ==========================================
# 모터 설정 함수
# ==========================================

def start_motors():

    for joint in wheel_joints:

        p.setJointMotorControl2(

            bodyUniqueId=robot,

            jointIndex=joint,

            controlMode=p.VELOCITY_CONTROL,

            targetVelocity=MOTOR_SPEED,

            force=MAX_TORQUE
        )


start_motors()


# ==========================================
# 실험 조건 출력
# ==========================================

print("\n==============================")
print("Experiment")
print("==============================")

print(
    f"Step height       : "
    f"{STEP_HEIGHT * 1000:.1f} mm"
)

print(
    f"Wheel radius      : "
    f"{WHEEL_RADIUS * 1000:.1f} mm"
)

print(
    f"h/r               : "
    f"{STEP_HEIGHT / WHEEL_RADIUS:.2f}"
)

print(
    f"Maximum torque    : "
    f"{MAX_TORQUE:.2f} N·m"
)

print(
    f"Simulation time   : "
    f"{SIMULATION_TIME:.1f} s"
)

print("==============================")

print("\n조작키")
print("R      : 처음부터 다시")
print("Space  : 일시정지 / 계속")
print("Q      : 종료")
print("==============================\n")


# ==========================================
# 초기 상태 저장
# ==========================================

initial_state = p.saveState()


# ==========================================
# 결과 변수
# ==========================================

max_pitch = 0.0
max_torque = 0.0

step = 0

paused = False
finished = False


# ==========================================
# 추가 계측: 원본 물리/제어 코드는 변경하지 않음
# ==========================================

front_contact_frames = 0
chassis_contact_frames = 0

first_front_contact = None
last_front_contact = None

first_chassis_contact_time = None
max_chassis_normal_force = 0.0


# ==========================================
# 결과 출력 함수
# ==========================================

def show_result():

    final_position, _ = (
        p.getBasePositionAndOrientation(
            robot
        )
    )

    final_x = final_position[0]


    # 문턱 안쪽까지 충분히 이동했으면 PASS
    PASS_POSITION = 0.45


    if final_x > PASS_POSITION:

        result = "PASS"

    else:

        result = "FAIL"


    print("\n==============================")
    print("RESULT")
    print("==============================")

    print(
        f"Step height     : "
        f"{STEP_HEIGHT * 1000:.1f} mm"
    )

    print(
        f"h/r             : "
        f"{STEP_HEIGHT / WHEEL_RADIUS:.2f}"
    )

    print(
        f"Result          : "
        f"{result}"
    )

    print(
        f"Maximum pitch   : "
        f"{max_pitch:.2f} deg"
    )

    print(
        f"Maximum torque  : "
        f"{max_torque:.3f} N·m"
    )

    print(
        f"Final X         : "
        f"{final_x:.3f} m"
    )

    print("--- CONTACT ANALYSIS ---")

    print(
        f"Front contact frames : "
        f"{front_contact_frames}"
    )

    print(
        f"First front contact  : "
        f"{first_front_contact}"
    )

    print(
        f"Last front contact   : "
        f"{last_front_contact}"
    )

    print(
        f"Chassis contact frames: "
        f"{chassis_contact_frames}"
    )

    print(
        f"First chassis contact: "
        f"{first_chassis_contact_time}"
    )

    print(
        f"Max chassis normal force: "
        f"{max_chassis_normal_force:.2f} N"
    )

    print("==============================\n")


# ==========================================
# 시뮬레이션
# ==========================================
analyzer = ContactMotionAnalyzer(
    robot=robot,
    plane_id=plane_id,
    step_id=step_id,
    wheel_joints=wheel_joints,
    time_step=TIME_STEP,
    filename="contact_mechanics_20mm.csv"
)

mechanics = ContactMechanicsAnalyzer(
    robot=robot,
    plane_id=plane_id,
    step_id=step_id,
    wheel_joints=wheel_joints,
    filename="contact_mechanics_20mm.csv"
)
while True:

    # --------------------------------------
    # 키보드 입력
    # --------------------------------------

    keys = p.getKeyboardEvents()


    # R = 처음부터 다시
    if (
        ord('r') in keys
        and
        keys[ord('r')] & p.KEY_WAS_TRIGGERED
    ):

        p.restoreState(
            initial_state
        )

        # 혹시 모르니 모터 명령도 다시 설정
        start_motors()

        step = 0

        max_pitch = 0.0
        max_torque = 0.0

        paused = False
        finished = False

        front_contact_frames = 0
        chassis_contact_frames = 0

        first_front_contact = None
        last_front_contact = None

        first_chassis_contact_time = None
        max_chassis_normal_force = 0.0

        print(
            "\nSimulation RESET\n"
        )


    # Space = 정지 / 계속
    if (
        32 in keys
        and
        keys[32] & p.KEY_WAS_TRIGGERED
    ):

        paused = not paused

        if paused:

            print("PAUSE")

        else:

            print("RESUME")


    # Q = 종료
    if (
        ord('q') in keys
        and
        keys[ord('q')] & p.KEY_WAS_TRIGGERED
    ):

        show_result()

        break


    # ======================================
    # 실제 물리 계산
    # ======================================

    if (
        not paused
        and
        not finished
    ):

        p.stepSimulation()

        

        step += 1
        if 5.0 <= step * TIME_STEP <= 6.5:
            mechanics.sample(
                sim_time=step * TIME_STEP,
                print_summary=(step % 24 == 0)
            )
        if 3.5 <= step * TIME_STEP <= SIMULATION_TIME:
            analyzer.sample(
                sim_time=step * TIME_STEP,
                print_summary=(step % 120 == 0)
            )

        ground_contacts = p.getContactPoints(
            bodyA=robot,
            bodyB=plane_id,
            linkIndexA=-1
         )

        if ground_contacts:
            print(
                f"[GROUND CONTACT] "
                f"time={step * TIME_STEP:.4f}s, "
                f"points={len(ground_contacts)}"
             )
        # ----------------------------------
        # 현재 시간
        # ----------------------------------

        current_time = (
            step * TIME_STEP
        )


        # ----------------------------------
        # 로봇 자세 측정
        # ----------------------------------

        position, orientation = (
            p.getBasePositionAndOrientation(
                robot
            )
        )


        roll, pitch, yaw = (
            p.getEulerFromQuaternion(
                orientation
            )
        )


        pitch_degree = abs(
            math.degrees(
                pitch
            )
        )


        if pitch_degree > max_pitch:

            max_pitch = pitch_degree


        # ----------------------------------
        # 바퀴 모터 토크 측정
        # ----------------------------------

        for joint in wheel_joints:

            joint_state = (
                p.getJointState(
                    robot,
                    joint
                )
            )


            motor_torque = abs(
                joint_state[3]
            )


            if motor_torque > max_torque:

                max_torque = motor_torque


        # ==================================
        # 추가: 접촉 데이터 측정
        # ==================================

        # 접촉 정보를 읽기만 하며 물리 상태는 변경하지 않음
        contacts = p.getContactPoints(
            bodyA=robot,
            bodyB=step_id
        )

        # 앞바퀴: Joint/Link 0, 1
        front_contacts = [
            c for c in contacts
            if c[3] in (0, 1)
        ]

        # 차체: Base Link -1
        chassis_contacts = [
            c for c in contacts
            if c[3] == -1
        ]

        if front_contacts:

            front_contact_frames += 1

            # 가장 큰 법선력을 가진 접촉점 기록
            c = max(
                front_contacts,
                key=lambda item: item[9]
            )

            sample = (
                round(current_time, 4),
                tuple(
                    round(v, 4)
                    for v in c[5]
                )
            )

            if first_front_contact is None:
                first_front_contact = sample

            last_front_contact = sample

        if chassis_contacts:

            chassis_contact_frames += 1

            if first_chassis_contact_time is None:
                first_chassis_contact_time = round(
                    current_time, 4
                )

            max_chassis_normal_force = max(
                max_chassis_normal_force,
                max(c[9] for c in chassis_contacts)
            )


        # ----------------------------------
        # 설정한 실험 시간이 지나면 정지
        # ----------------------------------

        if current_time >= SIMULATION_TIME:

            finished = True
            paused = True

            print(
                "\n실험 시간이 끝났습니다."
            )

            show_result()

            print(
                "R : 다시 시작"
            )

            print(
                "Q : 종료"
            )


    # 실제 시간과 비슷하게 표시
    time.sleep(
        TIME_STEP
    )


# ==========================================
# PyBullet 종료
# ==========================================
analyzer.close()
mechanics.close()
p.removeState(
    initial_state
)

p.disconnect()
