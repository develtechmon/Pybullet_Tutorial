"""
balance_env.py — final with yaw fix
Key changes from previous version:
  - Observation expanded from 5 to 7 elements (added yaw, yaw_rate)
  - Reward adds yaw_rate penalty to stop spinning

balance_env.py — FINAL VERSION
Reward components:
  1. Balance    — stay upright (pitch)
  2. Alive      — reward every step survived
  3. Effort     — efficient motor use
  4. Yaw        — stop spinning
  5. Position   — stop drifting left/right (NEW)

balance_env.py — FINAL VERSION
Fixed: avg_wheel_pos now relative to episode start
       prevents systematic drift in one direction

balance_env.py — FINAL VERSION with odometry
obs[4] = drift_distance from wheel odometry
Works identically in sim AND on real hardware (ESP32 encoders)
"""

# import gymnasium as gym
# from gymnasium import spaces
# import numpy as np
# import pybullet as p
# import pybullet_data
# import os


# class BalanceBotEnv(gym.Env):

#     metadata = {"render_modes": ["human", "rgb_array"]}

#     MAX_SPEED      = 34.87
#     MOTOR_DEADBAND = 0.10

#     PITCH_NOISE_STD      = 0.01
#     PITCH_RATE_NOISE_STD = 0.05

#     # Odometry constants — match real hardware
#     WHEEL_BASE   = 0.16     # metres between left and right wheel centres
#     WHEEL_RADIUS = 0.0425   # metres

#     FALL_ANGLE = 0.523
#     MAX_STEPS  = 2000
#     CONTROL_HZ = 100
#     PHYSICS_HZ = 1000
#     SUBSTEPS   = 10

#     def __init__(self, render_mode=None):
#         super().__init__()
#         self.render_mode = render_mode

#         # Odometry state — reset every episode
#         self.prev_left_pos  = 0.0
#         self.prev_right_pos = 0.0
#         self.est_x          = 0.0
#         self.est_y          = 0.0
#         self.heading        = 0.0

#         # 7 observations:
#         # [pitch, pitch_rate, left_vel, right_vel,
#         #  drift_distance, yaw, yaw_rate]
#         obs_low  = np.array([-np.pi, -20.0, -self.MAX_SPEED,
#                               -self.MAX_SPEED, 0.0,
#                               -np.pi, -10.0], dtype=np.float32)
#         obs_high = np.array([ np.pi,  20.0,  self.MAX_SPEED,
#                                self.MAX_SPEED, 10.0,
#                                np.pi,  10.0], dtype=np.float32)

#         self.observation_space = spaces.Box(obs_low, obs_high,
#                                             dtype=np.float32)
#         self.action_space = spaces.Box(
#             low=np.array([-1.0, -1.0], dtype=np.float32),
#             high=np.array([ 1.0,  1.0], dtype=np.float32)
#         )

#         self.physicsClient = None
#         self.robot         = None
#         self.step_count    = 0
#         self.urdf_path     = os.path.join(
#             os.path.dirname(os.path.abspath(__file__)), "balancebot.urdf"
#         )

#     def _setup_sim(self):
#         if self.physicsClient is not None:
#             p.resetSimulation(physicsClientId=self.physicsClient)
#             p.setGravity(0, 0, -9.81,
#                          physicsClientId=self.physicsClient)
#             p.setTimeStep(1.0 / self.PHYSICS_HZ,
#                           physicsClientId=self.physicsClient)
#             plane = p.loadURDF("plane.urdf",
#                                physicsClientId=self.physicsClient)
#             p.changeDynamics(plane, -1, lateralFriction=0.8,
#                              physicsClientId=self.physicsClient)
#             return

#         if self.render_mode == "human":
#             self.physicsClient = p.connect(p.GUI)
#             p.resetDebugVisualizerCamera(
#                 cameraDistance=0.8,
#                 cameraYaw=45,
#                 cameraPitch=-20,
#                 cameraTargetPosition=[0, 0, 0.1],
#                 physicsClientId=self.physicsClient
#             )
#         else:
#             self.physicsClient = p.connect(p.DIRECT)

#         p.setAdditionalSearchPath(
#             pybullet_data.getDataPath(),
#             physicsClientId=self.physicsClient
#         )
#         p.setTimeStep(1.0 / self.PHYSICS_HZ,
#                       physicsClientId=self.physicsClient)
#         p.setGravity(0, 0, -9.81,
#                      physicsClientId=self.physicsClient)

#         plane = p.loadURDF("plane.urdf",
#                            physicsClientId=self.physicsClient)
#         p.changeDynamics(plane, -1, lateralFriction=0.8,
#                          physicsClientId=self.physicsClient)

#     def _load_robot(self, init_pitch=0.0):
#         spawn_height = 0.0425
#         init_orn     = p.getQuaternionFromEuler([0, init_pitch, 0])

#         self.robot = p.loadURDF(
#             self.urdf_path,
#             basePosition=[0, 0, spawn_height],
#             baseOrientation=init_orn,
#             physicsClientId=self.physicsClient
#         )

#         for joint_idx in [1, 2]:
#             p.changeDynamics(
#                 self.robot, joint_idx,
#                 lateralFriction=0.8,
#                 rollingFriction=0.001,
#                 spinningFriction=0.001,
#                 physicsClientId=self.physicsClient
#             )
#             p.setJointMotorControl2(
#                 bodyUniqueId=self.robot,
#                 jointIndex=joint_idx,
#                 controlMode=p.VELOCITY_CONTROL,
#                 force=0,
#                 physicsClientId=self.physicsClient
#             )

#         # Record starting wheel positions for odometry
#         left_state  = p.getJointState(self.robot, 1,
#                                        physicsClientId=self.physicsClient)
#         right_state = p.getJointState(self.robot, 2,
#                                        physicsClientId=self.physicsClient)
#         self.prev_left_pos  = left_state[0]
#         self.prev_right_pos = right_state[0]

#         # Reset odometry state
#         self.est_x   = 0.0
#         self.est_y   = 0.0
#         self.heading = 0.0

#     def _update_odometry(self, left_pos, right_pos):
#         """
#         Differential drive odometry.
#         Same formula used on ESP32 with encoder counts.

#         delta_left/right = arc length each wheel travelled since last step
#         delta_dist       = forward distance (average of both wheels)
#         delta_heading    = rotation angle (difference between wheels)

#         est_x, est_y updated using dead reckoning.
#         """
#         delta_left  = (left_pos  - self.prev_left_pos)  * self.WHEEL_RADIUS
#         delta_right = (right_pos - self.prev_right_pos) * self.WHEEL_RADIUS

#         delta_dist    = (delta_left + delta_right) / 2.0
#         delta_heading = (delta_right - delta_left) / self.WHEEL_BASE

#         self.heading += delta_heading
#         self.est_x   += delta_dist * np.cos(self.heading)
#         self.est_y   += delta_dist * np.sin(self.heading)

#         self.prev_left_pos  = left_pos
#         self.prev_right_pos = right_pos

#     def _get_obs(self):
#         _, orn = p.getBasePositionAndOrientation(
#             self.robot, physicsClientId=self.physicsClient
#         )
#         euler      = p.getEulerFromQuaternion(orn)
#         pitch      = euler[1]
#         yaw        = euler[2]

#         _, ang_vel = p.getBaseVelocity(
#             self.robot, physicsClientId=self.physicsClient
#         )
#         pitch_rate = ang_vel[1]
#         yaw_rate   = ang_vel[2]

#         left_state  = p.getJointState(self.robot, 1,
#                                       physicsClientId=self.physicsClient)
#         right_state = p.getJointState(self.robot, 2,
#                                       physicsClientId=self.physicsClient)
#         left_vel  = left_state[1]
#         right_vel = right_state[1]

#         # Update odometry with current wheel positions
#         self._update_odometry(left_state[0], right_state[0])

#         # Euclidean distance from starting position
#         drift_distance = float(np.sqrt(self.est_x**2 + self.est_y**2))
#         drift_distance = min(drift_distance, 10.0)   # clip to obs bound

#         # MPU6050 noise — pitch sensors only
#         pitch      += np.random.normal(0, self.PITCH_NOISE_STD)
#         pitch_rate += np.random.normal(0, self.PITCH_RATE_NOISE_STD)

#         obs = np.array([
#             pitch,           # obs[0] — tilt angle (MPU6050 accel)
#             pitch_rate,      # obs[1] — tilt rate  (MPU6050 gyro)
#             left_vel,        # obs[2] — left wheel speed  (encoder)
#             right_vel,       # obs[3] — right wheel speed (encoder)
#             drift_distance,  # obs[4] — XY drift from start (odometry)
#             yaw,             # obs[5] — rotation angle (MPU6050 gyro integrated)
#             yaw_rate         # obs[6] — rotation rate  (MPU6050 gyro)
#         ], dtype=np.float32)

#         return np.clip(obs, self.observation_space.low,
#                        self.observation_space.high)

#     def _apply_action(self, action):
#         for joint_idx, raw_action in zip([1, 2], action):
#             if abs(raw_action) < self.MOTOR_DEADBAND:
#                 target_vel = 0.0
#             else:
#                 target_vel = float(raw_action) * self.MAX_SPEED

#             p.setJointMotorControl2(
#                 bodyUniqueId=self.robot,
#                 jointIndex=joint_idx,
#                 controlMode=p.VELOCITY_CONTROL,
#                 targetVelocity=target_vel,
#                 force=10.0,
#                 physicsClientId=self.physicsClient
#             )

#     def _compute_reward(self, obs, action):
#         pitch          = obs[0]
#         yaw_rate       = obs[6]
#         drift_distance = obs[4]

#         # 1. Balance — Gaussian, max=1.0 when upright
#         r_balance = float(np.exp(-10.0 * pitch ** 2))

#         # 2. Alive bonus — reward every step survived
#         r_alive = 0.1

#         # 3. Effort penalty — efficient motor use
#         r_effort = -0.01 * float(np.sum(np.square(action)))

#         # 4. Yaw penalty — stop spinning
#         r_yaw = -0.5 * float(yaw_rate ** 2)

#         # 5. Position penalty — odometry based, all directions
#         r_position = -0.1 * float(drift_distance ** 2)

#         return r_balance + r_alive + r_effort + r_yaw + r_position

#     def reset(self, seed=None, options=None):
#         super().reset(seed=seed)
#         self.step_count = 0
#         self._setup_sim()

#         init_pitch = np.random.uniform(
#             -0.087, 0.087   # ±5 degrees
#         )
#         self._load_robot(init_pitch=init_pitch)

#         for _ in range(10):
#             p.stepSimulation(physicsClientId=self.physicsClient)

#         return self._get_obs(), {}

#     def step(self, action):
#         self.step_count += 1
#         self._apply_action(action)

#         for _ in range(self.SUBSTEPS):
#             p.stepSimulation(physicsClientId=self.physicsClient)

#         obs        = self._get_obs()
#         pitch      = obs[0]
#         terminated = bool(abs(pitch) > self.FALL_ANGLE)
#         truncated  = self.step_count >= self.MAX_STEPS
#         reward     = self._compute_reward(obs, action)

#         if terminated:
#             reward -= 10.0

#         info = {"pitch_deg": float(np.degrees(pitch)),
#                 "step": self.step_count}
#         return obs, reward, terminated, truncated, info

#     def close(self):
#         if self.physicsClient is not None:
#             p.disconnect(self.physicsClient)
#             self.physicsClient = None


"""
balance_env.py — FINAL VERSION
================================

WHAT CHANGED FROM PREVIOUS VERSION:
  obs[4] = avg_wheel_vel  instead of drift_distance
  r_velocity = -0.1 * avg_wheel_vel²  instead of r_position

WHY:
  Odometry drift_distance is accumulated over many steps — delayed signal.
  Policy cannot connect old actions to delayed punishment.
  avg_wheel_vel is immediate — feedback every single step.
  Research confirmed: successful TWSBR RL uses velocity penalty not position.

SIM-TO-REAL:
  avg_wheel_vel = (left_encoder_delta + right_encoder_delta) / (2 * dt)
  Identical calculation on ESP32 encoders. No odometry drift accumulation.

OBSERVATION VECTOR (7 elements — same shape as before):
  obs[0] = pitch          MPU6050 accelerometer
  obs[1] = pitch_rate     MPU6050 gyroscope
  obs[2] = left_vel       left encoder rad/s
  obs[3] = right_vel      right encoder rad/s
  obs[4] = avg_wheel_vel  (left+right)/2 — replaces drift_distance
  obs[5] = yaw            heading angle
  obs[6] = yaw_rate       spin rate

REWARD (5 components):
  r_balance  = exp(-10 * pitch²)      stay upright
  r_alive    = +0.1                   survive each step
  r_effort   = -0.01 * actions²       efficient motors
  r_yaw      = -0.5 * yaw_rate²       stop spinning
  r_velocity = -0.1 * avg_wheel_vel²  stay stationary ← NEW
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import os


class BalanceBotEnv(gym.Env):

    metadata = {"render_modes": ["human", "rgb_array"]}

    MAX_SPEED      = 34.87
    MOTOR_DEADBAND = 0.10

    PITCH_NOISE_STD      = 0.01
    PITCH_RATE_NOISE_STD = 0.05

    FALL_ANGLE = 0.523
    MAX_STEPS  = 2000
    CONTROL_HZ = 100
    PHYSICS_HZ = 1000
    SUBSTEPS   = 10

    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        # obs[4] bounds now match avg_wheel_vel range (can be negative)
        obs_low  = np.array([-np.pi, -20.0, -self.MAX_SPEED,
                              -self.MAX_SPEED, -self.MAX_SPEED,
                              -np.pi, -10.0], dtype=np.float32)
        obs_high = np.array([ np.pi,  20.0,  self.MAX_SPEED,
                               self.MAX_SPEED,  self.MAX_SPEED,
                               np.pi,  10.0], dtype=np.float32)

        self.observation_space = spaces.Box(obs_low, obs_high,
                                            dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([ 1.0,  1.0], dtype=np.float32)
        )

        self.physicsClient = None
        self.robot         = None
        self.step_count    = 0
        self.urdf_path     = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "balancebot.urdf"
        )

    def _setup_sim(self):
        if self.physicsClient is not None:
            p.resetSimulation(physicsClientId=self.physicsClient)
            p.setGravity(0, 0, -9.81,
                         physicsClientId=self.physicsClient)
            p.setTimeStep(1.0 / self.PHYSICS_HZ,
                          physicsClientId=self.physicsClient)
            plane = p.loadURDF("plane.urdf",
                               physicsClientId=self.physicsClient)
            p.changeDynamics(plane, -1, lateralFriction=0.8,
                             physicsClientId=self.physicsClient)
            return

        if self.render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.8,
                cameraYaw=45,
                cameraPitch=-20,
                cameraTargetPosition=[0, 0, 0.1],
                physicsClientId=self.physicsClient
            )
        else:
            self.physicsClient = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self.physicsClient
        )
        p.setTimeStep(1.0 / self.PHYSICS_HZ,
                      physicsClientId=self.physicsClient)
        p.setGravity(0, 0, -9.81,
                     physicsClientId=self.physicsClient)

        plane = p.loadURDF("plane.urdf",
                           physicsClientId=self.physicsClient)
        p.changeDynamics(plane, -1, lateralFriction=0.8,
                         physicsClientId=self.physicsClient)

    def _load_robot(self, init_pitch=0.0):
        spawn_height = 0.0425
        init_orn     = p.getQuaternionFromEuler([0, init_pitch, 0])

        self.robot = p.loadURDF(
            self.urdf_path,
            basePosition=[0, 0, spawn_height],
            baseOrientation=init_orn,
            physicsClientId=self.physicsClient
        )

        for joint_idx in [1, 2]:
            p.changeDynamics(
                self.robot, joint_idx,
                lateralFriction=0.8,
                rollingFriction=0.001,
                spinningFriction=0.001,
                physicsClientId=self.physicsClient
            )
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                force=0,
                physicsClientId=self.physicsClient
            )

    def _get_obs(self):
        _, orn = p.getBasePositionAndOrientation(
            self.robot, physicsClientId=self.physicsClient
        )
        euler      = p.getEulerFromQuaternion(orn)
        pitch      = euler[1]
        yaw        = euler[2]

        _, ang_vel = p.getBaseVelocity(
            self.robot, physicsClientId=self.physicsClient
        )
        pitch_rate = ang_vel[1]
        yaw_rate   = ang_vel[2]

        left_state  = p.getJointState(self.robot, 1,
                                      physicsClientId=self.physicsClient)
        right_state = p.getJointState(self.robot, 2,
                                      physicsClientId=self.physicsClient)
        left_vel  = left_state[1]
        right_vel = right_state[1]

        # Average wheel velocity — immediate signal, no accumulation
        # Positive = moving forward, Negative = moving backward
        # ESP32 equivalent: (left_enc_delta + right_enc_delta) / (2 * dt)
        avg_wheel_vel = (left_vel + right_vel) / 2.0

        # MPU6050 noise on pitch sensors only
        pitch      += np.random.normal(0, self.PITCH_NOISE_STD)
        pitch_rate += np.random.normal(0, self.PITCH_RATE_NOISE_STD)

        obs = np.array([
            pitch,          # obs[0]
            pitch_rate,     # obs[1]
            left_vel,       # obs[2]
            right_vel,      # obs[3]
            avg_wheel_vel,  # obs[4] ← replaces drift_distance
            yaw,            # obs[5]
            yaw_rate        # obs[6]
        ], dtype=np.float32)

        return np.clip(obs, self.observation_space.low,
                       self.observation_space.high)

    def _apply_action(self, action):
        for joint_idx, raw_action in zip([1, 2], action):
            if abs(raw_action) < self.MOTOR_DEADBAND:
                target_vel = 0.0
            else:
                target_vel = float(raw_action) * self.MAX_SPEED

            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=10.0,
                physicsClientId=self.physicsClient
            )

    def _compute_reward(self, obs, action):
        pitch         = obs[0]
        yaw_rate      = obs[6]
        avg_wheel_vel = obs[4]

        # 1. Balance — max 1.0 when upright
        r_balance = float(np.exp(-10.0 * pitch ** 2))

        # 2. Alive bonus
        r_alive = 0.1

        # 3. Effort penalty
        r_effort = -0.01 * float(np.sum(np.square(action)))

        # 4. Yaw penalty — stop spinning
        r_yaw = -0.5 * float(yaw_rate ** 2)

        # 5. Velocity penalty — stay stationary ← KEY CHANGE
        # Immediate feedback every step — no accumulation delay
        r_velocity = -0.01 * float(avg_wheel_vel ** 2)

        return r_balance + r_alive + r_effort + r_yaw + r_velocity

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self._setup_sim()

        init_pitch = np.random.uniform(-0.087, 0.087)
        self._load_robot(init_pitch=init_pitch)

        for _ in range(10):
            p.stepSimulation(physicsClientId=self.physicsClient)

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self._apply_action(action)

        for _ in range(self.SUBSTEPS):
            p.stepSimulation(physicsClientId=self.physicsClient)

        obs        = self._get_obs()
        pitch      = obs[0]
        terminated = bool(abs(pitch) > self.FALL_ANGLE)
        truncated  = self.step_count >= self.MAX_STEPS
        reward     = self._compute_reward(obs, action)

        if terminated:
            reward -= 10.0

        info = {"pitch_deg": float(np.degrees(pitch)),
                "step": self.step_count}
        return obs, reward, terminated, truncated, info

    def close(self):
        if self.physicsClient is not None:
            p.disconnect(self.physicsClient)
            self.physicsClient = None