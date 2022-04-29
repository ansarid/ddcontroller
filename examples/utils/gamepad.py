#!/usr/bin/python3

'''
This file is part of the SCUTTLEPy library (https://github.com/ansarid/scuttlepy).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import time
import threading
import numpy as np
import inputs
from inputs import devices
from inputs import get_gamepad


class Gamepad:

    def __init__(self):

        gamepads = [device.name for device in devices if type(device) is inputs.GamePad]

        if gamepads:
            if 'ESM-9013' in gamepads:
                pass
            else:
                print('\nGamepad in incorrect mode.\n')
        else:
            print("\nNo gamepad detected.\n")
            return None
            # exit(1)

        self.axes_map = {
            'ABS_X':'LEFT_X',
            'ABS_Y':'LEFT_Y',
            'ABS_Z':'RIGHT_X',
            'ABS_RZ':'RIGHT_Y',
        }

        self.button_map = {
            'BTN_SOUTH':'Y',
            'BTN_EAST':'B',
            'BTN_C':'A',
            'BTN_NORTH':'X',
            'BTN_WEST':'LB',
            'BTN_Z':'RB',
            'BTN_TL':'LT',
            'BTN_TR':'RT',
            'BTN_TL2':'BACK',
            'BTN_TR2':'START',
            'BTN_SELECT':'L_JOY',
            'BTN_START':'R_JOY',
            'BTN_MODE':'MODE',
        }

        self.buttons = {}
        self.axes = {}
        self.hat = [0,0]
        self.states = { 'axes': self.axes,
                        'buttons': self.buttons,
                        'hat': self.hat
                    }

        for button in self.button_map.keys():
            self.buttons[self.button_map[button]] = 0

        for axis in self.axes_map.keys():
            self.axes[self.axes_map[axis]] = 128

        self.stopped = False

        self.state_updater_thread = threading.Thread(target=self.state_updater)
        self.state_updater_thread.start()

    def _get_states(self):
        events = get_gamepad()
        for event in events:
            # print(event.ev_type, event.code, event.state)
            if event.ev_type == "Absolute":
                if 'ABS_HAT' in event.code:
                    if 'ABS_HAT0X' == event.code:
                        self.hat[0] = event.state
                    elif 'ABS_HAT0Y' == event.code:
                        self.hat[1] = event.state
                else:
                    self.axes[self.axes_map[event.code]] = event.state
            elif event.ev_type == "Key":
                self.buttons[self.button_map[event.code]] = event.state
            else:
                pass

        self.states = {'axes': self.axes,
                       'buttons': self.buttons,
                       'hat': self.hat
                       }

        return self.states

    def state_updater(self):
        while not self.stopped:
            self._get_states()

    def get_states(self):
        return self.states

    def close(self):
        self.stopped = True
        self.state_updater_thread.join()

    def pressed(self, button):
        pass

    def unpressed(self, button):
        pass


if __name__ == "__main__":
    try:
        gamepad = Gamepad()
        while True:
            print(gamepad.axes, gamepad.buttons, gamepad.hat)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.close()
        # This code does not exit cleanly. Needs to be fixed.
