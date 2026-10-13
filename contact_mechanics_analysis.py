import csv
import math
import pybullet as p


class ContactMechanicsAnalyzer:
    """Read-only wheel/step contact geometry and estimated reaction moments."""

    def __init__(self, robot, plane_id, step_id, wheel_joints, filename):
        self.robot = robot
        self.plane_id = plane_id
        self.step_id = step_id
        self.wheel_joints = tuple(wheel_joints)
        self.file = open(filename, 'w', newline='', encoding='utf-8-sig')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_s', 'base_x_m', 'base_z_m', 'pitch_deg', 'base_vx_m_s',
            'front_step_contacts', 'rear_step_contacts',
            'front_step_normal_N', 'rear_step_normal_N',
            'front_step_force_x_N', 'front_step_force_z_N',
            'rear_step_force_x_N', 'rear_step_force_z_N',
            'front_step_moment_y_Nm', 'rear_step_moment_y_Nm',
            'all_step_moment_y_Nm', 'all_plane_moment_y_Nm',
            'all_contact_moment_y_Nm',
            'front_step_normal_x', 'front_step_normal_z',
            'rear_step_normal_x', 'rear_step_normal_z',
            'front_step_point_x_m', 'front_step_point_z_m',
            'rear_step_point_x_m', 'rear_step_point_z_m',
            'front_step_friction_N', 'rear_step_friction_N',
            'rear_plane_normal_N', 'chassis_plane_normal_N',
            'chassis_step_normal_N',
            'front_left_omega_rad_s', 'front_right_omega_rad_s',
            'rear_left_omega_rad_s', 'rear_right_omega_rad_s'
        ])

    @staticmethod
    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    @staticmethod
    def cross(a, b):
        return (a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0])

    @staticmethod
    def norm(v):
        return math.sqrt(sum(x*x for x in v))

    def _measure(self, contacts, links, reference_point):
        chosen = [c for c in contacts if c[3] in links and c[9] > 0]
        total_force = [0.0, 0.0, 0.0]
        total_moment = [0.0, 0.0, 0.0]
        normal_sum = 0.0
        friction_sum = 0.0
        normal_weighted = [0.0, 0.0, 0.0]
        point_weighted = [0.0, 0.0, 0.0]
        for c in chosen:
            # PyBullet: normalOnB (index 7) points from B toward A.
            # Lateral friction direction and magnitude are indices 11/10 and 13/12.
            # This reconstructs the contact reaction on body A (the robot).
            fn, n = c[9], c[7]
            f = [fn*n[k] + c[10]*c[11][k] + c[12]*c[13][k]
                 for k in range(3)]
            # Robot-side contact point (index 5), expressed in world coordinates.
            r = [c[5][k] - reference_point[k] for k in range(3)]
            m = self.cross(r, f)
            for k in range(3):
                total_force[k] += f[k]
                total_moment[k] += m[k]
                normal_weighted[k] += fn*n[k]
                point_weighted[k] += fn*c[5][k]
            normal_sum += fn
            friction_sum += math.hypot(c[10], c[12])
        if normal_sum:
            avg_normal = [x / normal_sum for x in normal_weighted]
            avg_point = [x / normal_sum for x in point_weighted]
        else:
            avg_normal = [float('nan')] * 3
            avg_point = [float('nan')] * 3
        return {
            'count': len(chosen), 'normal': normal_sum,
            'friction': friction_sum, 'force': total_force,
            'moment': total_moment, 'normal_direction': avg_normal,
            'point': avg_point
        }

    def sample(self, sim_time, print_summary=False):
        base_pos, base_quat = p.getBasePositionAndOrientation(self.robot)
        base_v, _ = p.getBaseVelocity(self.robot)
        # URDF base link's inertial frame can be offset from its link frame.
        # getBasePositionAndOrientation returns the base inertial/COM frame.
        base_com = base_pos
        pitch = math.degrees(p.getEulerFromQuaternion(base_quat)[1])
        sc = p.getContactPoints(bodyA=self.robot, bodyB=self.step_id)
        pc = p.getContactPoints(bodyA=self.robot, bodyB=self.plane_id)
        fs = self._measure(sc, (0, 1), base_com)
        rs = self._measure(sc, (2, 3), base_com)
        as_ = self._measure(sc, (-1, 0, 1, 2, 3), base_com)
        ap = self._measure(pc, (-1, 0, 1, 2, 3), base_com)
        rp = self._measure(pc, (2, 3), base_com)
        cp = self._measure(pc, (-1,), base_com)
        cs = self._measure(sc, (-1,), base_com)
        omegas = [p.getJointState(self.robot, j)[1] for j in self.wheel_joints]
        self.writer.writerow([
            round(sim_time, 5), base_pos[0], base_pos[2], pitch, base_v[0],
            fs['count'], rs['count'], fs['normal'], rs['normal'],
            fs['force'][0], fs['force'][2], rs['force'][0], rs['force'][2],
            fs['moment'][1], rs['moment'][1], as_['moment'][1],
            ap['moment'][1], as_['moment'][1] + ap['moment'][1],
            fs['normal_direction'][0], fs['normal_direction'][2],
            rs['normal_direction'][0], rs['normal_direction'][2],
            fs['point'][0], fs['point'][2], rs['point'][0], rs['point'][2],
            fs['friction'], rs['friction'], rp['normal'],
            cp['normal'], cs['normal'], *omegas
        ])
        if print_summary:
            print(
                f'[MECHANICS] t={sim_time:.3f}s '
                f'X={base_pos[0]:.3f}m Vx={base_v[0]:+.3f}m/s '
                f'pitch={pitch:+.2f}deg | '
                f'FrontStep N={fs["normal"]:.2f}N '
                f'Fx={fs["force"][0]:+.2f}N '
                f'My={fs["moment"][1]:+.3f}Nm | '
                f'RearStep N={rs["normal"]:.2f}N '
                f'Fx={rs["force"][0]:+.2f}N '
                f'My={rs["moment"][1]:+.3f}Nm | '
                f'AllContact My={as_["moment"][1]+ap["moment"][1]:+.3f}Nm'
            )

    def close(self):
        if not self.file.closed:
            self.file.close()
