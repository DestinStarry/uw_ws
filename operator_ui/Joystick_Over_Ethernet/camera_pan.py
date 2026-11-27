"""D-pad edge-controlled camera target (degrees, not measured servo position)."""
import math

class CameraPan:
    def __init__(self, step_deg=10.0, limit_deg=90.0):
        self.step = float(step_deg)
        self.limit = float(limit_deg)
        if not math.isfinite(self.step) or not math.isfinite(self.limit) or self.step <= 0 or self.limit <= 0:
            raise ValueError("camera step and limit must be finite and positive")
        self.angle_deg = 0.0
        self.previous = 0

    def update(self, hat_y):
        direction = 1 if hat_y > 0 else (-1 if hat_y < 0 else 0)
        if direction and direction != self.previous:
            self.angle_deg = max(-self.limit, min(self.limit, self.angle_deg + direction*self.step))
        self.previous = direction
        return self.angle_deg
