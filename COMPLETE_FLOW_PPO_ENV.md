<!-- model.learn()
    ↓
reset()
    ↓
_setup_sim()
    ↓
_load_robot()
    ↓
_get_obs()
    ↓
PPO receives obs
    ↓
PPO predicts action
    ↓
step(action)
    ↓
_apply_action(action)
    ↓
p.stepSimulation()
    ↓
_get_obs()
    ↓
_compute_reward(obs, action)
    ↓
return obs, reward, terminated, truncated, info
    ↓
PPO learns
    ↓
repeat -->

# User Guide: Reward Calculation and PPO–Environment Training Flow

## Purpose of This Guide

This guide explains two important parts of the balancing robot PPO training code:

1. **Reward calculation**
   - What each reward component means
   - Why each component is used
   - Step-by-step numerical examples
   - What is considered good or bad behavior
   - How the robot can reach reward `2100`

2. **Complete flow between PPO and the environment**
   - What happens when `model.learn()` starts
   - Which environment functions are called
   - How PPO interacts with `reset()`, `step()`, `_get_obs()`, `_apply_action()`, and `_compute_reward()`

---

# Part 1: Reward Calculation

## 1.1 The Reward Function

In the environment, reward is calculated inside:

```python
def _compute_reward(self, obs, action):
```

The reward formula is:

```python
r_balance = exp(-10 * pitch**2)
r_alive   = 0.1
r_effort  = -0.01 * sum(action**2)
r_yaw     = -0.5 * yaw_rate**2

reward = r_balance + r_alive + r_effort + r_yaw
```

If the robot falls, the code applies an extra penalty:

```python
reward -= 10.0
```

So the full reward idea is:

```text
Total reward =
    balance reward
  + alive reward
  - motor effort penalty
  - yaw spinning penalty
  - fall penalty if robot falls
```

---

## 1.2 What the Reward Is Trying to Teach

The reward function is the **teacher** for PPO.

PPO does not naturally know what balancing means. It only learns from reward:

```text
High reward  → repeat this behavior
Low reward   → avoid this behavior
```

The reward function teaches the robot:

| Behavior | Reward Effect | Meaning |
|---|---:|---|
| Standing upright | Positive | Good behavior |
| Staying alive longer | Positive | Good behavior |
| Using too much motor power | Negative | Bad / inefficient behavior |
| Spinning too much | Negative | Bad / unstable behavior |
| Falling | Large negative | Failure |

The goal is not only to stand upright. The goal is to:

```text
stand upright,
stay stable,
avoid spinning,
use smooth motor action,
and survive the full episode.
```

---

## 1.3 Observation and Action Used in Reward

The robot observation is:

```python
obs = [
    pitch,
    pitch_rate,
    yaw,
    yaw_rate
]
```

The reward mainly uses:

```python
pitch = obs[0]
yaw_rate = obs[3]
```

The action is:

```python
action = [
    left_motor_command,
    right_motor_command
]
```

Each motor command is between:

```text
-1.0 and +1.0
```

Example:

```python
action = [0.3, 0.3]
```

This means both wheels move forward with moderate power.

---

# 2. Reward Component 1: `r_balance`

## 2.1 Formula

```python
r_balance = exp(-10 * pitch**2)
```

Mathematically:

```text
r_balance = e^(-10 × pitch²)
```

Where:

```text
e ≈ 2.71828
```

Important:

```text
pitch must be in radians, not degrees.
```

---

## 2.2 What `r_balance` Means

`r_balance` rewards the robot for staying upright.

When pitch is close to zero:

```text
robot is upright
r_balance is close to 1.0
```

When pitch is large:

```text
robot is tilted
r_balance becomes smaller
```

The maximum value is:

```text
r_balance = 1.0
```

This happens when:

```text
pitch = 0
```

---

## 2.3 Why Use `pitch²`?

The code uses:

```python
pitch**2
```

because falling forward and falling backward should both be bad.

Example:

```text
pitch = +5 degrees
pitch = -5 degrees
```

Both should receive the same balance penalty.

Convert 5 degrees to radians:

```text
5 degrees ≈ 0.0873 radians
```

Square the value:

```text
(+0.0873)² = 0.0076
(-0.0873)² = 0.0076
```

So the reward becomes symmetric:

```text
forward tilt  → bad
backward tilt → bad
```

---

## 2.4 Why Use the Negative Sign?

The formula is:

```python
exp(-10 * pitch**2)
```

The negative sign makes the reward smaller when pitch becomes larger.

If the formula were wrong, for example:

```python
exp(+10 * pitch**2)
```

then the reward would increase when the robot tilts more. That would teach the robot the wrong behavior.

So:

```text
negative exponent = larger tilt gives smaller reward
```

---

## 2.5 Why Use `exp()`?

The exponential function gives a smooth reward curve.

It is useful because:

1. **Maximum reward is clear**

   When pitch is zero:

   ```text
   exp(0) = 1.0
   ```

2. **Small tilt gives small penalty**

   The robot is not punished too harshly for tiny movement.

3. **Large tilt gives strong penalty**

   As the robot tilts more, reward drops quickly.

4. **Reward stays positive**

   `exp()` always gives a value greater than zero.

5. **Smooth learning signal for PPO**

   PPO learns better when the reward changes smoothly instead of suddenly jumping.

---

## 2.6 Step-by-Step `r_balance` Calculations

### Example A: Pitch = 0 Degrees

Convert to radians:

```text
pitch = 0 degrees
pitch_rad = 0 × π / 180
pitch_rad = 0
```

Square the pitch:

```text
pitch² = 0² = 0
```

Multiply by `-10`:

```text
-10 × pitch² = -10 × 0 = 0
```

Apply exponential:

```text
r_balance = exp(0) = 1.0
```

Result:

```text
r_balance = 1.0
```

Meaning:

```text
Perfect upright position.
```

---

### Example B: Pitch = 5 Degrees

Convert to radians:

```text
pitch_rad = 5 × π / 180
pitch_rad ≈ 0.0873 rad
```

Square the pitch:

```text
pitch² = 0.0873²
pitch² ≈ 0.0076
```

Multiply by `-10`:

```text
-10 × pitch² = -10 × 0.0076
-10 × pitch² ≈ -0.0762
```

Apply exponential:

```text
r_balance = exp(-0.0762)
r_balance ≈ 0.9267
```

Result:

```text
r_balance ≈ 0.9267
```

Meaning:

```text
Still good, but not perfect.
```

---

### Example C: Pitch = 10 Degrees

Convert to radians:

```text
pitch_rad = 10 × π / 180
pitch_rad ≈ 0.1745 rad
```

Square the pitch:

```text
pitch² = 0.1745²
pitch² ≈ 0.0305
```

Multiply by `-10`:

```text
-10 × pitch² = -10 × 0.0305
-10 × pitch² ≈ -0.3046
```

Apply exponential:

```text
r_balance = exp(-0.3046)
r_balance ≈ 0.7374
```

Result:

```text
r_balance ≈ 0.7374
```

Meaning:

```text
The robot is still standing, but already risky.
```

---

### Example D: Pitch = 20 Degrees

Convert to radians:

```text
pitch_rad = 20 × π / 180
pitch_rad ≈ 0.3491 rad
```

Square the pitch:

```text
pitch² = 0.3491²
pitch² ≈ 0.1218
```

Multiply by `-10`:

```text
-10 × pitch² = -10 × 0.1218
-10 × pitch² ≈ -1.2185
```

Apply exponential:

```text
r_balance = exp(-1.2185)
r_balance ≈ 0.2957
```

Result:

```text
r_balance ≈ 0.2957
```

Meaning:

```text
Very bad balance. Robot is close to falling.
```

---

### Example E: Pitch = 30 Degrees

Convert to radians:

```text
pitch_rad = 30 × π / 180
pitch_rad ≈ 0.5236 rad
```

Square the pitch:

```text
pitch² = 0.5236²
pitch² ≈ 0.2742
```

Multiply by `-10`:

```text
-10 × pitch² = -10 × 0.2742
-10 × pitch² ≈ -2.7416
```

Apply exponential:

```text
r_balance = exp(-2.7416)
r_balance ≈ 0.0645
```

Result:

```text
r_balance ≈ 0.0645
```

Meaning:

```text
Almost no balance reward.
```

Also, the code defines fall condition around 30 degrees:

```python
FALL_ANGLE = 0.523
```

So near this point, the robot is considered fallen and receives an extra penalty:

```python
reward -= 10.0
```

---

## 2.7 `r_balance` Table

| Pitch Angle | Pitch Radians | pitch² | `-10 × pitch²` | `r_balance` | Meaning |
|---:|---:|---:|---:|---:|---|
| 0° | 0.0000 | 0.0000 | 0.0000 | 1.0000 | Perfect |
| 1° | 0.0175 | 0.0003 | -0.0030 | 0.9970 | Excellent |
| 2° | 0.0349 | 0.0012 | -0.0122 | 0.9879 | Very good |
| 3° | 0.0524 | 0.0027 | -0.0274 | 0.9730 | Very good |
| 4° | 0.0698 | 0.0049 | -0.0487 | 0.9524 | Good |
| 5° | 0.0873 | 0.0076 | -0.0762 | 0.9267 | Good |
| 10° | 0.1745 | 0.0305 | -0.3046 | 0.7374 | Risky |
| 15° | 0.2618 | 0.0685 | -0.6854 | 0.5039 | Bad |
| 20° | 0.3491 | 0.1218 | -1.2185 | 0.2957 | Very bad |
| 30° | 0.5236 | 0.2742 | -2.7416 | 0.0645 | Fall zone |

---

# 3. Reward Component 2: `r_alive`

## 3.1 Formula

```python
r_alive = 0.1
```

This means the robot receives:

```text
+0.1 reward every control step it is still alive
```

---

## 3.2 What `r_alive` Means

`r_alive` encourages the robot to survive longer.

Every time the robot completes one control step without the episode ending, it receives:

```text
+0.1
```

This teaches PPO:

```text
standing longer is better than falling early
```

---

## 3.3 Why Use `r_alive`?

Without `r_alive`, the robot may only care about momentary upright position.

With `r_alive`, the robot learns:

```text
Do not just be upright now.
Stay upright for as long as possible.
```

Your episode length is:

```python
MAX_STEPS = 2000
```

The control frequency is:

```text
100 Hz
```

So one full episode is:

```text
2000 steps / 100 Hz = 20 seconds
```

The alive bonus for a full episode is:

```text
0.1 × 2000 = 200
```

So the alive bonus strongly encourages the robot to survive the full episode.

---

# 4. Reward Component 3: `r_effort`

## 4.1 Formula

```python
r_effort = -0.01 * sum(action**2)
```

For the robot:

```python
action = [left_motor_command, right_motor_command]
```

So:

```text
sum(action²) = left_motor_command² + right_motor_command²
```

Therefore:

```text
r_effort = -0.01 × (left_action² + right_action²)
```

---

## 4.2 What `r_effort` Means

`r_effort` penalizes strong motor commands.

It tells PPO:

```text
Do not use more motor power than necessary.
```

This is important for real robot hardware because aggressive motor commands can cause:

```text
vibration,
gearbox stress,
battery drain,
motor heating,
unstable oscillation,
jerky movement.
```

---

## 4.3 What Does `sum(action²)` Mean?

Suppose:

```python
action = [0.4, 0.4]
```

Then:

```text
sum(action²) = 0.4² + 0.4²
             = 0.16 + 0.16
             = 0.32
```

So:

```text
r_effort = -0.01 × 0.32
         = -0.0032
```

This means PPO loses a small amount of reward for using motor power.

---

## 4.4 Why Square the Action?

The code uses:

```python
action**2
```

because positive and negative motor commands should both count as effort.

Example:

```text
action = +0.5
action = -0.5
```

Both use motor power.

Squaring gives:

```text
(+0.5)² = 0.25
(-0.5)² = 0.25
```

So forward power and backward power are penalized equally.

Squaring also penalizes large commands more strongly than small commands.

Example:

```text
0.2² = 0.04
1.0² = 1.00
```

A full-power command is much more expensive than a small correction.

---

## 4.5 Step-by-Step `r_effort` Calculations

### Example A: No Motor Effort

```python
action = [0.0, 0.0]
```

Square each action:

```text
left² = 0.0² = 0.0
right² = 0.0² = 0.0
```

Sum:

```text
sum(action²) = 0.0 + 0.0 = 0.0
```

Multiply:

```text
r_effort = -0.01 × 0.0
r_effort = 0.0
```

Result:

```text
No effort penalty.
```

---

### Example B: Small Motor Effort

```python
action = [0.2, 0.2]
```

Square each action:

```text
left² = 0.2² = 0.04
right² = 0.2² = 0.04
```

Sum:

```text
sum(action²) = 0.04 + 0.04 = 0.08
```

Multiply:

```text
r_effort = -0.01 × 0.08
r_effort = -0.0008
```

Result:

```text
Very small penalty.
```

Meaning:

```text
Small corrective actions are acceptable.
```

---

### Example C: Medium Motor Effort

```python
action = [0.5, 0.5]
```

Square each action:

```text
left² = 0.5² = 0.25
right² = 0.5² = 0.25
```

Sum:

```text
sum(action²) = 0.25 + 0.25 = 0.50
```

Multiply:

```text
r_effort = -0.01 × 0.50
r_effort = -0.005
```

Result:

```text
Small penalty.
```

Meaning:

```text
Acceptable if needed for recovery, but not ideal all the time.
```

---

### Example D: Maximum Motor Effort

```python
action = [1.0, 1.0]
```

Square each action:

```text
left² = 1.0² = 1.0
right² = 1.0² = 1.0
```

Sum:

```text
sum(action²) = 1.0 + 1.0 = 2.0
```

Multiply:

```text
r_effort = -0.01 × 2.0
r_effort = -0.02
```

Result:

```text
Penalty = -0.02
```

Meaning:

```text
The penalty is not huge, but it discourages PPO from always using full motor power.
```

---

## 4.6 `r_effort` Table

| Action | Calculation | `r_effort` | Meaning |
|---|---:|---:|---|
| `[0.0, 0.0]` | `-0.01 × (0² + 0²)` | `0.0000` | No effort |
| `[0.2, 0.2]` | `-0.01 × (0.04 + 0.04)` | `-0.0008` | Very small effort |
| `[0.5, 0.5]` | `-0.01 × (0.25 + 0.25)` | `-0.0050` | Medium effort |
| `[1.0, 1.0]` | `-0.01 × (1.00 + 1.00)` | `-0.0200` | Maximum effort |
| `[1.0, -1.0]` | `-0.01 × (1.00 + 1.00)` | `-0.0200` | Maximum effort, likely spinning |

---

# 5. Reward Component 4: `r_yaw`

## 5.1 Formula

```python
r_yaw = -0.5 * yaw_rate**2
```

Mathematically:

```text
r_yaw = -0.5 × yaw_rate²
```

---

## 5.2 What `r_yaw` Means

`r_yaw` penalizes spinning.

The robot may learn to balance but rotate around its vertical axis. That is not ideal.

So `r_yaw` tells PPO:

```text
Do not spin too fast.
```

---

## 5.3 Why Penalize Yaw Rate?

Without yaw penalty, PPO may discover a bad trick:

```text
balance while spinning continuously
```

That may look successful in reward from pitch alone, but it is bad for real robot behavior.

Yaw penalty encourages:

```text
stable upright balancing without unnecessary rotation
```

---

## 5.4 Why Square `yaw_rate`?

Squaring means both clockwise and counter-clockwise spinning are penalized.

Example:

```text
yaw_rate = +0.5 rad/s
yaw_rate = -0.5 rad/s
```

Both give:

```text
0.5² = 0.25
```

So both directions are penalized equally.

Squaring also makes fast spinning much worse than slow spinning.

---

## 5.5 Step-by-Step `r_yaw` Calculations

### Example A: No Spinning

```text
yaw_rate = 0.0 rad/s
```

Square:

```text
yaw_rate² = 0.0² = 0.0
```

Multiply:

```text
r_yaw = -0.5 × 0.0 = 0.0
```

Result:

```text
No yaw penalty.
```

---

### Example B: Small Spinning

```text
yaw_rate = 0.1 rad/s
```

Square:

```text
yaw_rate² = 0.1² = 0.01
```

Multiply:

```text
r_yaw = -0.5 × 0.01
r_yaw = -0.005
```

Result:

```text
Small penalty.
```

---

### Example C: Medium Spinning

```text
yaw_rate = 0.5 rad/s
```

Square:

```text
yaw_rate² = 0.5² = 0.25
```

Multiply:

```text
r_yaw = -0.5 × 0.25
r_yaw = -0.125
```

Result:

```text
Noticeable penalty.
```

---

### Example D: Fast Spinning

```text
yaw_rate = 1.0 rad/s
```

Square:

```text
yaw_rate² = 1.0² = 1.0
```

Multiply:

```text
r_yaw = -0.5 × 1.0
r_yaw = -0.5
```

Result:

```text
Large penalty.
```

---

## 5.6 `r_yaw` Table

| Yaw Rate | Calculation | `r_yaw` | Meaning |
|---:|---:|---:|---|
| `0.0 rad/s` | `-0.5 × 0.00` | `0.000` | No spinning |
| `0.1 rad/s` | `-0.5 × 0.01` | `-0.005` | Very small spinning |
| `0.3 rad/s` | `-0.5 × 0.09` | `-0.045` | Moderate spinning |
| `0.5 rad/s` | `-0.5 × 0.25` | `-0.125` | Bad spinning |
| `1.0 rad/s` | `-0.5 × 1.00` | `-0.500` | Very bad spinning |

---

# 6. Complete Step Reward Examples

The full reward is:

```text
reward = r_balance + r_alive + r_effort + r_yaw
```

If the robot falls:

```text
reward = reward - 10
```

---

## 6.1 Example 1: Perfect Upright

Given:

```text
pitch = 0 degrees
action = [0.0, 0.0]
yaw_rate = 0.0 rad/s
fallen = False
```

Calculate balance reward:

```text
pitch_rad = 0
r_balance = exp(-10 × 0²)
r_balance = 1.0
```

Alive reward:

```text
r_alive = 0.1
```

Effort penalty:

```text
r_effort = -0.01 × (0.0² + 0.0²)
r_effort = 0.0
```

Yaw penalty:

```text
r_yaw = -0.5 × 0.0²
r_yaw = 0.0
```

Total:

```text
reward = 1.0 + 0.1 + 0.0 + 0.0
reward = 1.1
```

Meaning:

```text
Excellent. This is the best possible step reward.
```

---

## 6.2 Example 2: Very Good Balance

Given:

```text
pitch = 2 degrees
action = [0.2, 0.2]
yaw_rate = 0.05 rad/s
fallen = False
```

Convert pitch:

```text
pitch_rad = 2 × π / 180
pitch_rad ≈ 0.0349
```

Balance reward:

```text
pitch² = 0.0349² ≈ 0.0012
-10 × pitch² ≈ -0.0122
r_balance = exp(-0.0122)
r_balance ≈ 0.9879
```

Alive reward:

```text
r_alive = 0.1
```

Effort penalty:

```text
r_effort = -0.01 × (0.2² + 0.2²)
r_effort = -0.01 × (0.04 + 0.04)
r_effort = -0.0008
```

Yaw penalty:

```text
r_yaw = -0.5 × 0.05²
r_yaw = -0.5 × 0.0025
r_yaw = -0.00125
```

Total:

```text
reward = 0.9879 + 0.1 - 0.0008 - 0.00125
reward ≈ 1.08585
```

Meaning:

```text
Very good. Robot is upright, calm, and uses small motor correction.
```

---

## 6.3 Example 3: Good but Not Enough for 2100

Given:

```text
pitch = 5 degrees
action = [0.4, 0.4]
yaw_rate = 0.1 rad/s
fallen = False
```

Balance reward:

```text
pitch_rad ≈ 0.0873
pitch² ≈ 0.0076
-10 × pitch² ≈ -0.0762
r_balance = exp(-0.0762)
r_balance ≈ 0.9267
```

Alive reward:

```text
r_alive = 0.1
```

Effort penalty:

```text
r_effort = -0.01 × (0.4² + 0.4²)
r_effort = -0.01 × (0.16 + 0.16)
r_effort = -0.0032
```

Yaw penalty:

```text
r_yaw = -0.5 × 0.1²
r_yaw = -0.5 × 0.01
r_yaw = -0.005
```

Total:

```text
reward = 0.9267 + 0.1 - 0.0032 - 0.005
reward ≈ 1.0185
```

Meaning:

```text
Good, but average reward is likely below the 2100 target if this continues for the whole episode.
```

---

## 6.4 Example 4: Risky Balance

Given:

```text
pitch = 10 degrees
action = [0.7, 0.7]
yaw_rate = 0.3 rad/s
fallen = False
```

Balance reward:

```text
pitch_rad ≈ 0.1745
pitch² ≈ 0.0305
-10 × pitch² ≈ -0.3046
r_balance = exp(-0.3046)
r_balance ≈ 0.7374
```

Alive reward:

```text
r_alive = 0.1
```

Effort penalty:

```text
r_effort = -0.01 × (0.7² + 0.7²)
r_effort = -0.01 × (0.49 + 0.49)
r_effort = -0.0098
```

Yaw penalty:

```text
r_yaw = -0.5 × 0.3²
r_yaw = -0.5 × 0.09
r_yaw = -0.045
```

Total:

```text
reward = 0.7374 + 0.1 - 0.0098 - 0.045
reward ≈ 0.7826
```

Meaning:

```text
Risky. Robot is tilted, using high motor command, and spinning somewhat.
```

---

## 6.5 Example 5: Bad Behavior

Given:

```text
pitch = 20 degrees
action = [1.0, 1.0]
yaw_rate = 0.8 rad/s
fallen = False
```

Balance reward:

```text
pitch_rad ≈ 0.3491
pitch² ≈ 0.1218
-10 × pitch² ≈ -1.2185
r_balance = exp(-1.2185)
r_balance ≈ 0.2957
```

Alive reward:

```text
r_alive = 0.1
```

Effort penalty:

```text
r_effort = -0.01 × (1.0² + 1.0²)
r_effort = -0.01 × 2.0
r_effort = -0.02
```

Yaw penalty:

```text
r_yaw = -0.5 × 0.8²
r_yaw = -0.5 × 0.64
r_yaw = -0.32
```

Total:

```text
reward = 0.2957 + 0.1 - 0.02 - 0.32
reward ≈ 0.0557
```

Meaning:

```text
Very bad. Robot is tilted, using maximum motor power, and spinning fast.
```

---

## 6.6 Example 6: Fallen Robot

Given:

```text
pitch = 31 degrees
action = [0.8, 0.8]
yaw_rate = 0.6 rad/s
fallen = True
```

The robot falls because pitch is greater than the fall threshold:

```python
abs(pitch) > FALL_ANGLE
```

The step reward before fall penalty may already be low. Then the code applies:

```python
reward -= 10.0
```

Example total:

```text
reward ≈ -10.0393
```

Meaning:

```text
Failure. Episode ends.
```

---

# 7. Full Reward Table

| Situation | Pitch | Action | Yaw Rate | Step Reward | Meaning |
|---|---:|---|---:|---:|---|
| Perfect upright | 0° | `[0.0, 0.0]` | 0.00 | 1.1000 | Excellent |
| Very good balance | 2° | `[0.2, 0.2]` | 0.05 | 1.0858 | Very good |
| Good / acceptable | 5° | `[0.4, 0.4]` | 0.10 | 1.0185 | Good, but likely below 2100 target |
| Weak / risky | 10° | `[0.7, 0.7]` | 0.30 | 0.7826 | Risky |
| Bad: tilted + spinning | 20° | `[1.0, 1.0]` | 0.80 | 0.0557 | Very bad |
| Failed: fallen | 31° | `[0.8, 0.8]` | 0.60 | about -10.0393 | Failure |

---

# 8. How to Achieve Reward 2100

The maximum episode length is:

```python
MAX_STEPS = 2000
```

The best possible step reward is approximately:

```text
r_balance = 1.0
r_alive   = 0.1
r_effort  = 0.0
r_yaw     = 0.0

best step reward = 1.1
```

Maximum episode reward:

```text
1.1 × 2000 = 2200
```

The stopping threshold is:

```python
reward_threshold = 2100
```

Average reward per step needed:

```text
2100 / 2000 = 1.05
```

So the robot must average about:

```text
1.05 reward per step
```

Because:

```text
r_alive = 0.1
```

The balance part after penalties must average around:

```text
1.05 - 0.1 = 0.95
```

So approximately:

```text
r_balance - effort_penalty - yaw_penalty ≈ 0.95
```

If effort and yaw penalties are very small, then:

```text
r_balance ≈ 0.95
```

Solve:

```text
exp(-10 × pitch²) = 0.95
```

Take natural log:

```text
-10 × pitch² = ln(0.95)
```

Since:

```text
ln(0.95) ≈ -0.0513
```

Then:

```text
pitch² = -ln(0.95) / 10
pitch² = 0.0513 / 10
pitch² = 0.00513
```

Square root:

```text
pitch = sqrt(0.00513)
pitch ≈ 0.0716 rad
```

Convert to degrees:

```text
0.0716 × 180 / π ≈ 4.1 degrees
```

Therefore:

```text
To reach around 2100, the robot must usually keep pitch around 4 degrees or less,
assuming yaw and effort penalties are small.
```

In real training, because yaw and effort penalties exist, the safer target is:

```text
average pitch around 0 to 3 degrees,
low yaw_rate,
smooth motor commands,
full 2000-step survival.
```

---

# 9. Good vs Bad Behavior Summary

## 9.1 Good Behavior

Good robot behavior looks like this:

```text
pitch near 0 degrees
pitch_rate small
yaw_rate small
motor commands smooth and not too high
survives full 2000 steps
```

Expected score:

```text
Around 2100 to 2200
```

---

## 9.2 Acceptable Behavior

Acceptable behavior:

```text
pitch around 4 to 5 degrees
small yaw_rate
moderate motor effort
survives most or all of the episode
```

Expected score:

```text
Around 2000 to 2100
```

It may balance, but may not pass the 2100 threshold consistently.

---

## 9.3 Bad Behavior

Bad behavior:

```text
pitch above 10 degrees often
yaw_rate high
motor commands near maximum
robot oscillates or shakes
robot falls early
```

Expected score:

```text
Low reward, often far below 2100
```

---

# Part 2: Complete Flow Between PPO and Environment

# 10. What PPO Needs From the Environment

For PPO training with Stable-Baselines3, the custom environment must provide:

```text
1. observation_space
2. action_space
3. reset()
4. step(action)
5. close()
```

Your environment also uses helper functions:

```text
_setup_sim()
_load_robot()
_get_obs()
_apply_action()
_compute_reward()
```

PPO does not call all helper functions directly. Instead:

```text
PPO calls reset() and step(action)
reset() and step(action) call the helper functions
```

---

# 11. Main Training Code

The main training starts here:

```python
model.learn(
    total_timesteps=500_000,
    callback=[eval_callback, checkpoint_callback, vecnorm_callback],
    progress_bar=True
)
```

This line begins the relationship between PPO and the robot environment.

---

# 12. High-Level PPO–Environment Flow

The high-level flow is:

```text
model.learn()
    ↓
environment.reset()
    ↓
PPO receives observation
    ↓
PPO chooses action
    ↓
environment.step(action)
    ↓
environment returns new observation, reward, done flags, info
    ↓
PPO stores experience
    ↓
repeat many times
    ↓
PPO updates neural network
    ↓
continue until total_timesteps reached or reward threshold achieved
```

---

# 13. Environment Setup Before Training

Before training, the code checks the environment:

```python
env = BalanceBotEnv()
check_env(env)
env.close()
```

This verifies that the environment follows Gymnasium rules.

During this check, the checker may call:

```text
reset()
step()
close()
```

This is like an inspection before real training begins.

---

# 14. Training Environment Creation

The training environment is created:

```python
train_env = DummyVecEnv([lambda: BalanceBotEnv()])
```

Then wrapped with normalization:

```python
train_env = VecNormalize(
    train_env,
    norm_obs=True,
    norm_reward=True,
    clip_obs=10.0,
    clip_reward=10.0
)
```

So PPO actually interacts with:

```text
PPO
 ↓
VecNormalize
 ↓
DummyVecEnv
 ↓
BalanceBotEnv
```

Meaning:

```text
PPO does not directly talk to raw BalanceBotEnv.
It talks through wrappers.
```

---

# 15. What `VecNormalize` Does

`VecNormalize` normalizes observations and rewards.

This helps the neural network learn more stably because raw values can have different scales.

Example observation:

```text
pitch      = 0.05
pitch_rate = 2.5
yaw        = 1.0
yaw_rate   = 0.3
```

These values have different ranges. Normalization helps convert them into a more learning-friendly scale.

Important:

```text
The trained PPO model depends on the saved VecNormalize statistics.
```

That is why the code saves:

```text
vecnormalize.pkl
```

---

# 16. Complete Function Flow During One Episode

When a new episode starts:

```text
PPO / VecNormalize / DummyVecEnv
    ↓
BalanceBotEnv.reset()
```

Inside `reset()`:

```text
reset()
    ↓
step_count = 0
    ↓
_setup_sim()
    ↓
_load_robot(init_pitch=random tilt)
    ↓
stepSimulation() for settling
    ↓
_get_obs()
    ↓
return obs, info
```

Then PPO receives the first observation:

```text
obs = [pitch, pitch_rate, yaw, yaw_rate]
```

PPO then chooses an action:

```text
action = [left_motor_command, right_motor_command]
```

Then the environment performs one control step:

```text
step(action)
    ↓
step_count += 1
    ↓
_apply_action(action)
    ↓
stepSimulation() repeated SUBSTEPS times
    ↓
_get_obs()
    ↓
_compute_reward(obs, action)
    ↓
check terminated
    ↓
check truncated
    ↓
return obs, reward, terminated, truncated, info
```

---

# 17. Detailed Explanation of Each Function

## 17.1 `__init__()`

Purpose:

```text
Prepare the environment rules and initial variables.
```

It defines:

```text
observation_space
action_space
robot variables
physics client
URDF path
step counter
```

Story meaning:

```text
This is where the training world is described before the robot starts learning.
```

---

## 17.2 `reset()`

Purpose:

```text
Start a new episode.
```

Called when:

```text
training begins
robot falls
episode reaches max steps
evaluation episode begins
```

Typical flow:

```text
reset step_count to 0
setup simulation
load robot
randomize starting pitch
settle physics
return first observation
```

Story meaning:

```text
The robot is placed back on the floor for a new attempt.
```

---

## 17.3 `_setup_sim()`

Purpose:

```text
Create or reset the PyBullet simulation world.
```

It sets:

```text
gravity
physics timestep
ground plane
friction
GUI or DIRECT mode
```

Story meaning:

```text
This prepares the training arena.
```

---

## 17.4 `_load_robot()`

Purpose:

```text
Load the robot URDF into the simulation.
```

It also:

```text
sets initial pitch
places wheels at correct ground height
configures wheel friction
disables default motor damping
```

Story meaning:

```text
This places the robot into the arena.
```

---

## 17.5 `_get_obs()`

Purpose:

```text
Read the robot state and return observation.
```

It returns:

```text
[pitch, pitch_rate, yaw, yaw_rate]
```

These are equivalent to IMU-based measurements:

```text
pitch      → accelerometer / orientation
pitch_rate → gyroscope Y axis
yaw        → integrated gyroscope Z axis
yaw_rate   → gyroscope Z axis
```

Story meaning:

```text
This is the robot checking its inner ear.
```

---

## 17.6 `_apply_action(action)`

Purpose:

```text
Convert PPO action into motor velocity commands.
```

PPO outputs:

```text
action values between -1.0 and +1.0
```

The function converts them into:

```text
wheel target velocity
```

It also applies motor deadband:

```text
if command is too small, motor does not move
```

Story meaning:

```text
This is where PPO's decision becomes real wheel movement.
```

---

## 17.7 `_compute_reward(obs, action)`

Purpose:

```text
Calculate how good or bad the robot's current behavior is.
```

It uses:

```text
pitch
yaw_rate
action magnitude
```

It returns:

```text
reward value
```

Story meaning:

```text
This is the teacher giving marks.
```

---

## 17.8 `step(action)`

Purpose:

```text
Advance the simulation by one control step.
```

It does:

```text
apply action
run physics
read new observation
calculate reward
check if robot fell
check if episode time ended
return results to PPO
```

Story meaning:

```text
This is one moment of the robot's life.
```

---

## 17.9 `close()`

Purpose:

```text
Cleanly close the PyBullet simulation.
```

Story meaning:

```text
This shuts down the training room.
```

---

# 18. Full PPO Training Flow

```text
START train_ppo.py
    ↓
Create temporary BalanceBotEnv
    ↓
check_env(env)
    ↓
close temporary env
    ↓
Create training environment
    ↓
Wrap with DummyVecEnv
    ↓
Wrap with VecNormalize
    ↓
Create evaluation environment
    ↓
Wrap evaluation environment
    ↓
Create callbacks
    ↓
Create PPO model
    ↓
model.learn()
        ↓
        train_env.reset()
            ↓
            VecNormalize.reset()
                ↓
                DummyVecEnv.reset()
                    ↓
                    BalanceBotEnv.reset()
                        ↓
                        _setup_sim()
                        ↓
                        _load_robot()
                        ↓
                        _get_obs()
        ↓
        PPO receives normalized observation
        ↓
        PPO chooses action
        ↓
        train_env.step(action)
            ↓
            VecNormalize.step()
                ↓
                DummyVecEnv.step()
                    ↓
                    BalanceBotEnv.step(action)
                        ↓
                        _apply_action(action)
                        ↓
                        p.stepSimulation() repeated SUBSTEPS times
                        ↓
                        _get_obs()
                        ↓
                        _compute_reward(obs, action)
                        ↓
                        check terminated
                        ↓
                        check truncated
                        ↓
                        return obs, reward, terminated, truncated, info
        ↓
        PPO stores experience
        ↓
        callbacks run
        ↓
        repeat until n_steps collected
        ↓
        PPO updates policy network and value network
        ↓
        continue until 500,000 timesteps or reward threshold achieved
    ↓
Save final model
    ↓
Save vecnormalize.pkl
    ↓
Close environments
END
```

---

# 19. What Happens When the Robot Falls

The robot falls when:

```python
abs(pitch) > FALL_ANGLE
```

With:

```python
FALL_ANGLE = 0.523
```

This is about:

```text
30 degrees
```

When it falls:

```text
terminated = True
reward -= 10.0
episode ends
new reset() happens
```

Flow:

```text
step(action)
    ↓
robot tilt exceeds fall angle
    ↓
terminated = True
    ↓
fall penalty applied
    ↓
return result to PPO
    ↓
new episode begins
    ↓
reset()
```

---

# 20. What Happens When the Robot Survives Full Episode

The episode reaches time limit when:

```python
step_count >= MAX_STEPS
```

With:

```python
MAX_STEPS = 2000
```

Then:

```text
truncated = True
```

This means:

```text
The robot did not fall.
The episode ended because the time limit was reached.
```

This is good.

Flow:

```text
step(action)
    ↓
step_count reaches 2000
    ↓
truncated = True
    ↓
episode ends successfully
    ↓
reset()
```

---

# 21. PPO Update Flow

PPO does not update the neural network after every single step.

In the training code:

```python
n_steps = 2048
```

So PPO does:

```text
collect 2048 environment steps
    ↓
calculate advantages
    ↓
update policy network and value network
    ↓
collect another 2048 steps
    ↓
update again
```

This repeats until:

```text
total_timesteps = 500,000
```

or until the reward threshold is reached.

---

# 22. Callback Flow During Training

Three callbacks are used:

```python
callback=[eval_callback, checkpoint_callback, vecnorm_callback]
```

---

## 22.1 Evaluation Callback

The evaluation callback runs every:

```python
eval_freq = 5000
```

It performs:

```text
run evaluation episodes
calculate mean reward
save best model if performance improves
stop training if mean reward exceeds 2100
```

The stopping condition is:

```python
StopTrainingOnRewardThreshold(reward_threshold=2100)
```

This means:

```text
If the model averages more than 2100 reward during evaluation, training can stop.
```

---

## 22.2 Checkpoint Callback

The checkpoint callback saves model backups every:

```python
save_freq = 10000
```

Example saved files:

```text
./models/checkpoints/balancebot_ppo_10000_steps.zip
./models/checkpoints/balancebot_ppo_20000_steps.zip
```

Purpose:

```text
If training crashes or is stopped, you still have saved models.
```

---

## 22.3 VecNormalize Callback

The custom callback saves normalization statistics every:

```python
save_freq = 10000
```

It saves:

```text
./models/vecnormalize.pkl
```

Purpose:

```text
The trained model needs the same normalization statistics during testing and deployment.
```

Without the correct `vecnormalize.pkl`, the model may receive wrongly scaled observations and perform poorly.

---

# 23. Final Files After Training

At the end of training, the code saves:

```python
model.save("./models/balancebot_ppo_final")
train_env.save("./models/vecnormalize.pkl")
```

Expected output files:

```text
Best model : ./models/best/best_model.zip
Final model: ./models/balancebot_ppo_final.zip
Norm stats : ./models/vecnormalize.pkl
```

Meaning:

| File | Purpose |
|---|---|
| `best_model.zip` | Best model from evaluation |
| `balancebot_ppo_final.zip` | Final model after training ends |
| `vecnormalize.pkl` | Observation/reward normalization statistics |

---

# 24. Complete Story Summary

The training story is:

```text
The robot starts slightly tilted.
It reads pitch, pitch_rate, yaw, and yaw_rate.
PPO chooses left and right wheel commands.
The robot moves in PyBullet physics.
The environment calculates reward.
PPO learns whether the action was good or bad.
The robot repeats this many times.
Eventually, PPO learns how to keep the robot upright.
```

The climax of learning happens when PPO discovers:

```text
If the robot leans forward, move wheels forward.
If the robot leans backward, move wheels backward.
If the robot spins, correct wheel difference.
Use only enough motor power to stay stable.
```

The ending is successful when:

```text
The robot survives close to 2000 steps,
keeps pitch near 0 to 3 degrees,
avoids spinning,
uses smooth motor action,
and achieves evaluation reward above 2100.
```

---

# 25. Final Key Points

Remember these core ideas:

```text
r_balance rewards upright posture.
r_alive rewards survival.
r_effort penalizes unnecessary motor power.
r_yaw penalizes spinning.
fall penalty strongly punishes failure.
```

To reach reward `2100`, the robot should:

```text
survive the full 20-second episode,
keep average pitch around 0 to 4 degrees,
keep yaw_rate low,
use smooth motor commands,
and avoid falling.
```

The PPO–environment relationship is:

```text
PPO chooses action.
Environment simulates result.
Environment gives reward.
PPO learns from reward.
Repeat until the robot becomes stable.
```
