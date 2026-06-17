"""
balance_env_v2.py — v2_non_encoder_robot
==========================================

VERSION HISTORY:
  v1_encoder_robot:     7 observations including wheel velocity from encoders
  v2_non_encoder_robot: 4 observations — IMU only, no encoders needed

WHAT CHANGED FROM v1:
  REMOVED observations:
    obs[2] = left_vel       (requires encoder)
    obs[3] = right_vel      (requires encoder)
    obs[4] = avg_wheel_vel  (requires encoder)

  KEPT observations:
    obs[0] = pitch          MPU6050 accelerometer
    obs[1] = pitch_rate     MPU6050 gyroscope Y axis
    obs[2] = yaw            MPU6050 gyroscope Z integrated  ← was obs[5]
    obs[3] = yaw_rate       MPU6050 gyroscope Z axis        ← was obs[6]

  REMOVED reward component:
    r_velocity = -0.01 * avg_wheel_vel²  (no encoder = can't measure)

  UPDATED physical parameters:
    MAX_SPEED    = 17.8     (225RPM × 0.758 L298N voltage correction)
    WHEEL_RADIUS = 0.03405m  (68.1mm diameter / 2, caliper measured)
    WHEEL_BASE   = 0.203m    (physically measured centre-to-centre)
    spawn_height = 0.03405m  (= wheel radius)
    Body height  = 0.10m     (10cm confirmed)

WHY 4 OBSERVATIONS WORK WITHOUT ENCODERS:
  Classical PID controllers for TWSBR only use pitch and pitch_rate.
  Adding yaw and yaw_rate helps the policy resist spinning.
  The Epersist paper (arXiv:2207.11431) used this exact observation
  set with ESP32 + MPU6050 — no encoders — and achieved real hardware
  balancing successfully.

TRADE-OFF vs v1:
  v1 (7 obs):  balances AND stays near starting position
  v2 (4 obs):  balances well, may drift slowly — acceptable for v2

SIM-TO-REAL MAPPING:
  obs[0] pitch      → MPU6050 accel: atan2(ax, az)
  obs[1] pitch_rate → MPU6050 gyro:  gy × DEG_TO_RAD
  obs[2] yaw        → MPU6050 gyro:  integrated gz × DEG_TO_RAD × dt
  obs[3] yaw_rate   → MPU6050 gyro:  gz × DEG_TO_RAD

MPU6050 PLACEMENT (hardware):
  Mount at TOP of 18cm frame, perfectly centred left-to-right
  X axis → forward/backward
  Y axis → left/right
  Z axis → up
  Keep away from motors and battery (vibration + magnetic interference)
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import os


class BalanceBotEnv(gym.Env):

    metadata = {"render_modes": ["human", "rgb_array"]}

    # ── Motor: Cytron SPG30-20K ────────────────────────────────
    # 225 RPM × (2π/60) = 23.56 rad/s no-load at 12V
    # Through L298N: 9.1V → 23.56 × (9.1/12) = 17.8 rad/s
    MAX_SPEED      = 17.8
    MOTOR_DEADBAND = 0.10    # real motors don't respond below 10% PWM

    # ── MPU6050 noise (from datasheet) ────────────────────────
    PITCH_NOISE_STD      = 0.01    # accelerometer RMS noise (rad)
    PITCH_RATE_NOISE_STD = 0.05    # gyroscope RMS noise (rad/s)

    # ── Episode parameters ────────────────────────────────────
    FALL_ANGLE = 0.523    # 30 degrees — terminate episode
    MAX_STEPS  = 2000     # 20 seconds at 100Hz
    PHYSICS_HZ = 1000     # physics simulation frequency
    SUBSTEPS   = 10       # physics steps per control step (100Hz control)

    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        # ── 4 observations — IMU only, no encoders ────────────
        # All directly readable from MPU6050 on real hardware
        obs_low  = np.array([-np.pi, -20.0,
                              -np.pi, -10.0], dtype=np.float32)
        obs_high = np.array([ np.pi,  20.0,
                               np.pi,  10.0], dtype=np.float32)

        self.observation_space = spaces.Box(obs_low, obs_high,
                                            dtype=np.float32)

        # Two wheel velocity commands in [-1, 1]
        # Scaled to MAX_SPEED internally in _apply_action
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([ 1.0,  1.0], dtype=np.float32)
        )

        self.physicsClient = None
        self.robot         = None
        self.step_count    = 0
        self.urdf_path     = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "balancebot.urdf"
        )

    def _setup_sim(self):
        if self.physicsClient is not None:
            # Reuse existing connection — no GUI reconnect noise
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
        # Spawn height = wheel radius so wheels sit on ground
        spawn_height = 0.034
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
            # Disable default motor damping for clean velocity control
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                force=0,
                physicsClientId=self.physicsClient
            )

    def _get_obs(self):
        # ── Orientation from PyBullet ──────────────────────────
        # Equivalent: MPU6050 accelerometer + gyroscope
        _, orn = p.getBasePositionAndOrientation(
            self.robot, physicsClientId=self.physicsClient
        )
        euler      = p.getEulerFromQuaternion(orn)
        pitch      = euler[1]    # forward/backward tilt
        yaw        = euler[2]    # heading rotation

        # ── Angular velocity from PyBullet ────────────────────
        # Equivalent: MPU6050 gyroscope raw output
        _, ang_vel = p.getBaseVelocity(
            self.robot, physicsClientId=self.physicsClient
        )
        pitch_rate = ang_vel[1]  # tilt rate rad/s
        yaw_rate   = ang_vel[2]  # spin rate rad/s

        # ── Add MPU6050 sensor noise ──────────────────────────
        # Models real sensor noise from datasheet
        # Only pitch sensors need noise — gyro Z is relatively clean
        pitch      += np.random.normal(0, self.PITCH_NOISE_STD)
        pitch_rate += np.random.normal(0, self.PITCH_RATE_NOISE_STD)

        obs = np.array([
            pitch,       # obs[0] — MPU6050 accelerometer
            pitch_rate,  # obs[1] — MPU6050 gyroscope Y
            yaw,         # obs[2] — MPU6050 gyroscope Z integrated
            yaw_rate     # obs[3] — MPU6050 gyroscope Z
        ], dtype=np.float32)

        return np.clip(obs, self.observation_space.low,
                       self.observation_space.high)

    def _apply_action(self, action):
        for joint_idx, raw_action in zip([1, 2], action):
            # Motor deadband — SPG30 won't respond below 10% PWM
            if abs(raw_action) < self.MOTOR_DEADBAND:
                target_vel = 0.0
            else:
                target_vel = float(raw_action) * self.MAX_SPEED

            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=target_vel,
                force=0.128,   # SPG30 max torque = 128mNm = 0.128 N·m
                physicsClientId=self.physicsClient
            )

    def _compute_reward(self, obs, action):
        pitch    = obs[0]
        yaw_rate = obs[3]

        # 1. Balance — Gaussian centred at pitch=0
        #    Max = 1.0 when perfectly upright
        #    Drops sharply as robot tilts
        r_balance = float(np.exp(-15.0 * pitch ** 2))

        # 2. Alive bonus — reward every step survived
        r_alive = 0.1

        # 3. Effort penalty — discourage unnecessary motor commands
        #    Protects real gearbox, saves battery
        r_effort = -0.01 * float(np.sum(np.square(action)))

        # 4. Yaw penalty — discourage spinning
        #    Quadratic: fast spin penalised heavily
        r_yaw = -0.5 * float(yaw_rate ** 2)

        # NOTE: No velocity/position penalty in v2
        # No encoders = no wheel velocity measurement
        # Robot may drift slowly — acceptable for v2 hardware testing

        return r_balance + r_alive + r_effort + r_yaw

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self._setup_sim()

        # Randomise starting tilt ±5°
        # Forces policy to learn recovery, not just standing still
        init_pitch = np.random.uniform(-0.087, 0.087)
        self._load_robot(init_pitch=init_pitch)

        # Let physics settle before returning first observation
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.physicsClient)

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self._apply_action(action)

        # Run physics at 1000Hz, control at 100Hz
        for _ in range(self.SUBSTEPS):
            p.stepSimulation(physicsClientId=self.physicsClient)

        obs        = self._get_obs()
        pitch      = obs[0]
        terminated = bool(abs(pitch) > self.FALL_ANGLE)
        truncated  = self.step_count >= self.MAX_STEPS
        reward     = self._compute_reward(obs, action)

        if terminated:
            reward -= 10.0   # large fall penalty

        info = {"pitch_deg": float(np.degrees(pitch)),
                "step": self.step_count}
        return obs, reward, terminated, truncated, info

    def close(self):
        if self.physicsClient is not None:
            p.disconnect(self.physicsClient)
            self.physicsClient = None