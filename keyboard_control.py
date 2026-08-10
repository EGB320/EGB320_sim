"""Small Windows keyboard helper used by the teleoperation example."""

import ctypes
import os


_KEY_CODES = {
    'w': 0x57,
    'a': 0x41,
    's': 0x53,
    'd': 0x44,
    'space': 0x20,
    'q': 0x51,
}


class KeyboardController:
    """Read held keys and key-press edges without an extra Python package."""

    def __init__(self):
        if os.name != 'nt':
            raise RuntimeError('The keyboard example currently requires Windows.')

        self._get_key_state = ctypes.windll.user32.GetAsyncKeyState
        self._get_key_state.argtypes = [ctypes.c_int]
        self._get_key_state.restype = ctypes.c_short
        self._previous_states = {}

    def is_down(self, key):
        """Return True while a key is held down."""
        return bool(self._get_key_state(_KEY_CODES[key]) & 0x8000)

    def was_pressed(self, key):
        """Return True once when a key changes from released to pressed."""
        is_down = self.is_down(key)
        was_down = self._previous_states.get(key, False)
        self._previous_states[key] = is_down
        return is_down and not was_down

    def drive_command(self, forward_speed, turn_speed):
        """Return ``(forward_velocity, turn_velocity)`` for W/A/S/D."""
        forward = int(self.is_down('w')) - int(self.is_down('s'))
        turn = int(self.is_down('a')) - int(self.is_down('d'))
        return forward_speed * forward, turn_speed * turn


def clear_console():
    """Clear the terminal used by the keyboard status display."""
    os.system('cls' if os.name == 'nt' else 'clear')
