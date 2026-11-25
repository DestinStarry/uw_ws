"""Offline UI preview: replaces BOTH networking and camera, never sends controls."""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pygame
import main as ui


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="validation/ui_preview.png")
    parser.add_argument("--invalid", action="store_true")
    args = parser.parse_args()
    values = {"ARMED": "0", "CONTROL_MODE": "OPEN", "DEPTH_HOLD": "0",
              "DEPTH": "0.026", "DEPTH_VALID": "1", "DEPTH_AGE_MS": "24",
              "IMU_OK": "1", "IMU_ZEROED": "1", "YAW": "12.3", "PITCH": "-1.2", "ROLL": "0.4",
              "VOLT": "15.23", "VOLT_VALID": "1", "VOLT_AGE_MS": "90",
              "CPU_TEMP": "42.5", "CPU_TEMP_VALID": "1", "CPU_TEMP_AGE_MS": "100",
              "CH0": "1200", "CH0_VALID": "1", "CH1": "0", "CH1_VALID": "1",
              "CH2": "--", "CH2_VALID": "0", "CH3": "--", "CH3_VALID": "0"}
    if args.invalid:
        values.update(CPU_TEMP_VALID="0", VOLT_VALID="0", DEPTH_VALID="0", IMU_OK="0", CH0_VALID="0", CH1_VALID="0")
    class FakeClient:
        connected = True
        connection_status = "模拟预览 / 未发送控制"
        def __init__(self, *args): pass
        def connect(self): pass
        def send(self, _): return True
        def close(self): pass
        def get(self, key, default=None): return values.get(key, default)
        def telemetry_snapshot(self): return dict(values), {key: .05 for key in values}
    class FakeVideo:
        def __init__(self, *_): self.surface = None
        def start(self):
            self.surface = pygame.Surface((ui.VIDEO_WIDTH, ui.VIDEO_HEIGHT))
            width = (ui.VIDEO_WIDTH - 16) // 2
            self.surface.fill((24, 24, 24))
            font = pygame.font.SysFont("arial", 32)
            for i, color in enumerate(((35, 80, 115), (40, 100, 95))):
                rect = pygame.Rect(i * (width + 16), 0, width, ui.VIDEO_HEIGHT)
                pygame.draw.rect(self.surface, color, rect)
                text = font.render(("LEFT" if i == 0 else "RIGHT") + " CAMERA - SIMULATED", True, (220, 235, 240))
                self.surface.blit(text, text.get_rect(center=rect.center))
        def get_surface(self): return self.surface
        def get_fps(self): return 30.0
        def stop(self): pass
        def join(self, **_): pass
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ui.CommandClient = FakeClient
    ui.VideoReceiver = FakeVideo
    pygame.joystick.get_count = lambda: 0
    pygame.display.get_desktop_sizes = lambda: [(1920, 1080)]
    pygame.event.get = lambda: [pygame.event.Event(pygame.QUIT)]
    original_flip = pygame.display.flip
    def capture():
        pygame.image.save(pygame.display.get_surface(), str(output))
        original_flip()
    pygame.display.flip = capture
    original_print = ui.TextPrint.print
    def checked_print(self, screen, text, color=(0,0,0)):
        if self.x == 1086:
            rect = self.font.render(text, True, color).get_rect(topleft=(self.x, self.y))
            assert rect.right <= 1580 and rect.bottom <= 790, (text, rect)
        original_print(self, screen, text, color)
    ui.TextPrint.print = checked_print
    ui.main()
    print("\nSaved offline preview:", output)


if __name__ == "__main__":
    main()
