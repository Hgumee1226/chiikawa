import csv
import math
import pybullet as p


class ContactMotionAnalyzer:
    """Read-only contact diagnostics for a fixed step and a four-wheel robot."""

    def __init__(self, robot, plane_id, step_id, wheel_joints, time_step,
                 filename='contact_motion_19mm.csv'):
        self.robot = robot
        self.plane_id = plane_id
        self.step_id = step_id
        self.wheel_joints = list(wheel_joints)
        self.dt = time_step
        self.file = open(filename, 'w', newline='', encoding='utf-8-sig')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_s', 'base_x_m', 'base_z_m', 'base_vx_m_s', 'pitch_deg',
            'front_step_normal_N', 'front_step_friction_N',
            'front_step_max_slip_m_s', 'front_step_mean_slip_m_s',
            'rear_plane_normal_N', 'rear_plane_friction_N',
            'rear_plane_max_slip_m_s', 'rear_step_normal_N',
            'rear_step_max_slip_m_s', 'chassis_plane_normal_N',
            'chassis_step_normal_N',
            'front_left_omega_rad_s', 'front_right_omega_rad_s',
            'rear_left_omega_rad_s', 'rear_right_omega_rad_s',
            'front_left_torque_Nm', 'front_right_torque_Nm',
            'rear_left_torque_Nm', 'rear_right_torque_Nm'
        ])

    @staticmethod
    def _dot(a, b):
        return sum(x*y for x, y in zip(a, b))

    @staticmethod
    def _cross(a, b):
        return (a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0])

    def _contact_slip(self, contact, link_kinematics):
        # The plane and step are stationary. c[5] is the world contact
        # point on robot; c[7] is the contact normal on body B.
        link = contact[3]
        if link not in link_kinematics:
            return 0.0
        com, v, omega = link_kinematics[link]
        r = tuple(contact[5][i] - com[i] for i in range(3))
        spin_v = self._cross(omega, r)
        point_v = tuple(v[i] + spin_v[i] for i in range(3))
        normal = contact[7]
        normal_v = self._dot(point_v, normal)
        tangential_v = tuple(point_v[i] - normal_v*normal[i] for i in range(3))
        return math.sqrt(self._dot(tangential_v, tangential_v))

    def sample(self, sim_time, print_summary=False):
        position, orientation = p.getBasePositionAndOrientation(self.robot)
        velocity, _ = p.getBaseVelocity(self.robot)
        pitch = math.degrees(p.getEulerFromQuaternion(orientation)[1])

        link_kinematics = {}
        for link in self.wheel_joints:
            state = p.getLinkState(self.robot, link, computeLinkVelocity=1)
            # state[0] = world COM position; state[6:8] = COM linear/angular velocity
            link_kinematics[link] = (state[0], state[6], state[7])

        plane_contacts = p.getContactPoints(bodyA=self.robot, bodyB=self.plane_id)
        step_contacts = p.getContactPoints(bodyA=self.robot, bodyB=self.step_id)

        def measure(contacts, links):
            selected = [c for c in contacts if c[3] in links]
            normal = sum(c[9] for c in selected)
            # PyBullet contact tuple: lateral friction forces c[10], c[12].
            friction = sum(math.hypot(c[10], c[12]) for c in selected)
            slips = [self._contact_slip(c, link_kinematics) for c in selected]
            return normal, friction, max(slips, default=0.0), (
                sum(slips)/len(slips) if slips else 0.0
            )

        fs = measure(step_contacts, (0, 1))
        rp = measure(plane_contacts, (2, 3))
        rs = measure(step_contacts, (2, 3))
        cp = measure(plane_contacts, (-1,))
        cs = measure(step_contacts, (-1,))
        joint_states = [p.getJointState(self.robot, j) for j in self.wheel_joints]
        omegas = [s[1] for s in joint_states]
        torques = [s[3] for s in joint_states]
        self.writer.writerow([
            round(sim_time, 5), position[0], position[2], velocity[0], pitch,
            fs[0], fs[1], fs[2], fs[3],
            rp[0], rp[1], rp[2], rs[0], rs[2], cp[0], cs[0],
            *omegas, *torques
        ])
        if print_summary:
            print(
                f'[MOTION] t={sim_time:.2f}s X={position[0]:.3f}m '
                f'Vx={velocity[0]:+.3f}m/s pitch={pitch:+.2f}deg | '
                f'FrontStep N={fs[0]:.1f}N F={fs[1]:.1f}N '
                f'slip={fs[2]:.3f}m/s | '
                f'RearPlane N={rp[0]:.1f}N slip={rp[2]:.3f}m/s | '
                f'RearStep N={rs[0]:.1f}N slip={rs[2]:.3f}m/s | '
                f'ChassisPlane N={cp[0]:.1f}N | '
                f'omega={[round(v, 2) for v in omegas]} '
                f'torque={[round(v, 3) for v in torques]}'
            )

    def close(self):
        if not self.file.closed:
            self.file.close()
