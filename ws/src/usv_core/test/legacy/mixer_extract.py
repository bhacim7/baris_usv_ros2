import numpy as np

import config as cfg


# Servo channel assignment, read once from config. These used to be inlined as
# getattr(cfg, 'SOL_MOTOR', 1) etc. with fallbacks (1,3,2,4,5) that did not match
# either config.py (6,3,5,2,7) or MainSystem2's channel list.
CH_REAR_LEFT = getattr(cfg, 'SOL_MOTOR', 1)
CH_REAR_RIGHT = getattr(cfg, 'SAG_MOTOR', 3)
CH_FRONT_LEFT = getattr(cfg, 'FRONT_SOL_MOTOR', 2)
CH_FRONT_RIGHT = getattr(cfg, 'FRONT_SAG_MOTOR', 4)
CH_STEER = getattr(cfg, 'FRONT_STEER_SERVO', 5)


def _slew(target, prev, max_delta):
    """Limit one channel's per-cycle change to +/- max_delta."""
    if max_delta is None or max_delta <= 0 or prev is None:
        return int(target)
    if target > prev + max_delta:
        return int(prev + max_delta)
    if target < prev - max_delta:
        return int(prev - max_delta)
    return int(target)


def apply_motor_mixer(controller, forward_pwm, yaw_pwm, slew=True):
    """
    Control Mixer: Distributes Forward Effort and Yaw Effort across 5 channels.
    Implements Steering Deadband for smooth turns and push-pull differential.

    `slew=False` bypasses the rate limiter - use it for emergency braking and for the
    SIGTERM neutralise, where an instantaneous command is the whole point.
    """
    base = getattr(cfg, 'BASE_PWM', 1500)

    # 1. Yaw Effort mappings
    # Map Yaw to Steering Servo (Direct proportion)
    steer_out = base + yaw_pwm
    steer_max = getattr(cfg, 'STEER_MAX_PWM', 1900)
    steer_min = getattr(cfg, 'STEER_MIN_PWM', 1100)
    steer_out = np.clip(steer_out, steer_min, steer_max)

    # 2. Differential Thrust (Continuous micro-corrections, removed deadband)
    # Applying differential thrust continuously prevents lateral drift by fighting wind/current smoothly

    # Check if we are doing a pure spot turn (stationary)
    is_spot_turn = (forward_pwm == 1500 and yaw_pwm != 0)

    if is_spot_turn:
        # Disable dynamic multiplier for pure spot turns so we don't multiply max outputs
        diff_thrust = yaw_pwm * 1.0
    else:
        # Dynamic multiplier: Boosts differential thrust at lower speeds where the steering servo is hydrodynamically weak.
        # At 1500 (stopped), multiplier is 2.0. At 1700 (CRUISE_PWM=200), multiplier is ~1.4. At 1850+ (max speed), multiplier is 1.0.
        diff_multiplier = 1.0 + max(0.0, (1850.0 - forward_pwm) / 350.0)
        diff_thrust = yaw_pwm * diff_multiplier

    # 3. Calculate Individual Thrusters
    # Evasive Braking override: If we are actively braking (reverse thrust),
    # we DO NOT want positive differential thrust pushing us forward on the outside.
    if forward_pwm < 1450:  # Actively reversing
        rear_left = int(forward_pwm)
        rear_right = int(forward_pwm)
        front_left = int(forward_pwm)
        front_right = int(forward_pwm)
    elif is_spot_turn:
        # Pure Pivot: Front and rear motors must use identical differential thrust
        # to ensure the boat twists perfectly around its center without moving backward.
        rear_left = int(forward_pwm + diff_thrust)
        rear_right = int(forward_pwm - diff_thrust)
        front_left = int(forward_pwm + diff_thrust)
        front_right = int(forward_pwm - diff_thrust)
    else:
        # Standard turning (forward movement)
        # Right turn (yaw_pwm > 0): Left motors spin faster, Right motors slower
        rear_left = int(forward_pwm + diff_thrust)
        rear_right = int(forward_pwm - diff_thrust)

        # Front thrusters (keep forward effort high, apply smaller diff to prevent crab-walking)
        front_left = int(forward_pwm + (diff_thrust * 0.5))
        front_right = int(forward_pwm - (diff_thrust * 0.5))

    # Clamp all thrusters
    rear_left = int(np.clip(rear_left, 1100, 1900))
    rear_right = int(np.clip(rear_right, 1100, 1900))
    front_left = int(np.clip(front_left, 1100, 1900))
    front_right = int(np.clip(front_right, 1100, 1900))
    steer_out = int(steer_out)

    # --- Slew-rate limiting (cfg.MAX_PWM_CHANGE) ---
    # This limit has been sitting in config.py unused since the beginning. Flight data
    # shows why it matters: at 23:48:08 the mixer went 1100/1900 -> 1831/1528 -> 1900/1109
    # in 0.25 s, an ~800 PWM reversal on every thruster inside a quarter second. That is
    # not control, it is chatter - and with the hull's ~1 s yaw time constant the boat
    # simply keeps rotating through it. The previous PWM is read back from the controller's
    # own cache, so no extra state has to be threaded through here.
    if slew:
        max_delta = getattr(cfg, 'MAX_PWM_CHANGE', 60)
        rear_left = _slew(rear_left, controller.get_servo_pwm(CH_REAR_LEFT), max_delta)
        rear_right = _slew(rear_right, controller.get_servo_pwm(CH_REAR_RIGHT), max_delta)
        front_left = _slew(front_left, controller.get_servo_pwm(CH_FRONT_LEFT), max_delta)
        front_right = _slew(front_right, controller.get_servo_pwm(CH_FRONT_RIGHT), max_delta)
        steer_out = _slew(steer_out, controller.get_servo_pwm(CH_STEER), max_delta)

    # Command the hardware
    force = not slew  # emergency / neutralise paths must not be rate limited downstream
    controller.set_servo(CH_REAR_LEFT, rear_left, force=force)
    controller.set_servo(CH_REAR_RIGHT, rear_right, force=force)
    controller.set_servo(CH_FRONT_LEFT, front_left, force=force)
    controller.set_servo(CH_FRONT_RIGHT, front_right, force=force)
    controller.set_servo(CH_STEER, steer_out, force=force)
