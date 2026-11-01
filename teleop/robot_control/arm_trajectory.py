"""Bounded joint command trajectories, independent of DDS and IK."""
import numpy as np


class ArmTrajectoryLimiter:
    def __init__(self, acceleration_limit, tracking_error_limit, max_dt=0.02):
        limits = np.array([acceleration_limit, tracking_error_limit, max_dt], dtype=float)
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
            raise ValueError("Arm trajectory limits must be positive and finite")
        self.acceleration_limit = float(acceleration_limit)
        self.tracking_error_limit = float(tracking_error_limit)
        self.max_dt = float(max_dt)
        self.command = None
        self.velocity = None

    def reset(self, position):
        position = np.asarray(position, dtype=float)
        if not np.all(np.isfinite(position)):
            raise ValueError("Cannot anchor a trajectory to a non-finite position")
        self.command = position.copy()
        self.velocity = np.zeros_like(position)

    def hold(self):
        """Freeze the last command and discard velocity after a feedback interruption."""
        if self.velocity is not None:
            self.velocity.fill(0.0)
        return None if self.command is None else self.command.copy()

    def step(self, target, measured, dt, velocity_limit):
        target = np.asarray(target, dtype=float)
        measured = np.asarray(measured, dtype=float)
        if target.shape != measured.shape or not np.all(np.isfinite(target)):
            raise ValueError("Invalid joint target")
        if not np.all(np.isfinite(measured)):
            raise ValueError("Invalid joint feedback")
        dt = float(dt)
        velocity_limit = float(velocity_limit)
        if not np.isfinite(dt) or dt <= 0 or not np.isfinite(velocity_limit) or velocity_limit <= 0:
            raise ValueError("Period and velocity limit must be positive and finite")
        dt = min(dt, self.max_dt)
        if self.command is None or self.command.shape != measured.shape:
            self.reset(measured)

        # A blocked joint must not accumulate an arbitrarily distant command.
        lower = measured - self.tracking_error_limit
        upper = measured + self.tracking_error_limit
        anchor = np.clip(self.command, lower, upper)
        reanchored = anchor != self.command
        self.velocity[reanchored] = 0.0
        self.command = anchor

        error = target - self.command
        dv = self.acceleration_limit * dt
        # Exact discrete stopping distance: with n remaining braking steps,
        # distance = dt * (n*v - dv*n*(n-1)/2). This avoids overshooting a
        # fixed target while retaining the acceleration bound near arrival.
        distance_units = np.abs(error) / (self.acceleration_limit * dt * dt)
        n = np.maximum(1.0, np.ceil((np.sqrt(1.0 + 8.0 * distance_units) - 1.0) / 2.0))
        braking_velocity = dv * (distance_units / n + (n - 1.0) / 2.0)
        desired_velocity = np.sign(error) * np.minimum(velocity_limit, braking_velocity)
        velocity = np.clip(desired_velocity, self.velocity - dv, self.velocity + dv)
        candidate = self.command + velocity * dt
        command = np.clip(candidate, lower, upper)

        # Tracking and feedback interlocks take priority over normal trajectory
        # acceleration. Clear velocity at the boundary to prevent windup.
        velocity[command != candidate] = 0.0
        self.command = command
        self.velocity = velocity
        return command.copy()
