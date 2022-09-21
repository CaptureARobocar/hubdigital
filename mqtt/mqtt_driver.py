from __future__ import annotations

from base64 import b64decode
from json import dumps, loads
from typing import Optional, Union, Callable, Final

import numpy as np
import pygame
from cv2 import imdecode, resize, imshow, waitKey, putText, IMREAD_COLOR, FONT_HERSHEY_COMPLEX_SMALL
from numpy import ndarray, frombuffer, uint8
from paho.mqtt.client import Client

# DEFINE SPECIAL KEYS
_TOP_ARROW_CHR: Final[chr] = chr(0)
_BOTTOM_ARROW_CHR: Final[chr] = chr(1)
_LEFT_ARROW_CHR: Final[chr] = chr(2)
_RIGHT_ARROW_CHR: Final[chr] = chr(3)
_ESC_CHR: Final[chr] = chr(27)
_SPACE_CHR: Final[chr] = ' '

# DEFINE CONTROL KEYS
_EXIT_KEYS: Final[tuple[chr, ...]] = (_ESC_CHR,)
_FORWARD_KEYS: Final[tuple[chr, ...]] = (_TOP_ARROW_CHR, 'Z')
_BACKWARD_KEYS: Final[tuple[chr, ...]] = (_BOTTOM_ARROW_CHR, 'S')
_RESET_THROTTLE_KEYS: Final[tuple[chr, ...]] = (' ',)
_RIGHT_KEYS: Final[tuple[chr, ...]] = (_RIGHT_ARROW_CHR, 'D')
_LEFT_KEYS: Final[tuple[chr, ...]] = (_LEFT_ARROW_CHR, 'Q')
_RESET_ANGLE_KEYS: Final[tuple[chr, ...]] = ('\t',)
_PILOT_MODE_KEYS: Final[tuple[chr, ...]] = ('P',)
_USER_MODE_KEYS: Final[tuple[chr, ...]] = ('U',)
_HELP_KEYS: Final[tuple[chr, ...]] = ('H',)
_USE_JOYSTICKS: Final[tuple[chr, ...]] = ('J',)
_USE_RECORD: Final[tuple[chr, ...]] = ('R',)

# DEFINE KEY ACTION MAPPING
_KEY_ACTION: Final[dict[chr, Callable[[DkMqttDriver], None]]] = {
    **dict.fromkeys(_EXIT_KEYS, lambda mqtt_driver: mqtt_driver.exit()),
    **dict.fromkeys(_FORWARD_KEYS, lambda mqtt_driver: mqtt_driver.increase_throttle()),
    **dict.fromkeys(_BACKWARD_KEYS, lambda mqtt_driver: mqtt_driver.decrease_throttle()),
    **dict.fromkeys(_RESET_THROTTLE_KEYS, lambda mqtt_driver: mqtt_driver.reset_throttle()),
    **dict.fromkeys(_RIGHT_KEYS, lambda mqtt_driver: mqtt_driver.turn_right()),
    **dict.fromkeys(_LEFT_KEYS, lambda mqtt_driver: mqtt_driver.turn_left()),
    **dict.fromkeys(_RESET_ANGLE_KEYS, lambda mqtt_driver: mqtt_driver.reset_angle()),
    **dict.fromkeys(_PILOT_MODE_KEYS, lambda mqtt_driver: mqtt_driver.use_pilot_mode()),
    **dict.fromkeys(_USER_MODE_KEYS, lambda mqtt_driver: mqtt_driver.use_user_mode()),
    **dict.fromkeys(_HELP_KEYS, lambda mqtt_driver: mqtt_driver.toggle_help()),
    **dict.fromkeys(_USE_JOYSTICKS, lambda mqtt_driver: mqtt_driver.toggle_joystick_mode()),
    **dict.fromkeys(_USE_RECORD, lambda mqtt_driver: mqtt_driver.toggle_record_mode()),
}

# DEFINE JSON KEYS
_ANGLE_JSON_KEY: Final[str] = 'angle'
_THROTTLE_JSON_KEY: Final[str] = 'throttle'
_MODE_JSON_KEY: Final[str] = 'drive_mode'
_USER_MODE_JSON_VALUE: Final[str] = 'user'
_PILOT_MODE_JSON_VALUE: Final[str] = 'pilot'
_RECORD_MODE_JSON_KEY: Final[str] = 'recording'

# IMAGE CONSTANTS
_IMG_NAME: str = 'vehicle_cam'
_MARGIN_LEFT: int = 10


# PRIVATE FUNCTION
def _put_text(
        img_array: ndarray,
        text: str,
        org: tuple[int, int],
        color: tuple[int, int, int],
        font_face: int = FONT_HERSHEY_COMPLEX_SMALL,
        font_scale: int = 1,
        thickness: int = 2
) -> ndarray:
    return putText(img_array, text, org=org, fontFace=font_face, fontScale=font_scale, color=color, thickness=thickness)


# PUBLIC CLASS
class DkMqttDriver:
    def __init__(
            self,
            video_topic: str,
            ctrl_topic: str,
            host: str = '127.0.0.1',
            port: int = 1883,
            username: Optional[str] = None,
            password: Optional[bytes] = None,
            frame_size: tuple[int, int] = (1024, 720),
            throttle_precision: float = 0.25,
            angle_precision: float = 0.25
    ):
        assert 0 < throttle_precision <= 1, "throttle_precision must be between 0 and 1 (included)"
        assert 0 < angle_precision <= 1, "angle_precision must be between 0 and 1 (included)"
        self._frame_size: tuple[int, int] = frame_size
        self._display_loading_image(pending_topic=video_topic)
        self._mqtt_client: Client = Client()
        if username is not None:
            self._mqtt_client.username_pw_set(username, password)
        self._mqtt_client.connect(host, port)
        self._display_help: bool = False
        self._mqtt_client.on_message = self.on_message
        self._mqtt_client.subscribe(video_topic)
        self._ctrl_topic: str = ctrl_topic
        self._throttle_precision: float = throttle_precision
        self._angle_precision: float = angle_precision
        self._init_joysticks()
        self._ctrl_data: dict[str, Union[float, str]] = {
            _ANGLE_JSON_KEY: 0.,
            _THROTTLE_JSON_KEY: 0.,
            _MODE_JSON_KEY: _USER_MODE_JSON_VALUE,
            _RECORD_MODE_JSON_KEY: False
        }
        self._mqtt_client.loop_forever()

    def on_message(self, client, userdata, message) -> None:
        img: Final[bytes] = b64decode(loads(message.payload)['data'])
        img_array: ndarray = frombuffer(img, uint8)
        img_array = imdecode(img_array, IMREAD_COLOR)
        img_array = resize(img_array, self._frame_size)
        img_array = self._insert_driving_data(img_array)
        imshow(_IMG_NAME, img_array)
        ctrl_key: chr = chr(waitKey(1) & 0xFF)
        self._run_ctrl(ctrl_key)

    def increase_throttle(self) -> None:
        self._ctrl_data[_THROTTLE_JSON_KEY] = min(1., self._ctrl_data[_THROTTLE_JSON_KEY] + self._throttle_precision)

    def decrease_throttle(self) -> None:
        self._ctrl_data[_THROTTLE_JSON_KEY] = max(-1., self._ctrl_data[_THROTTLE_JSON_KEY] - self._throttle_precision)

    def reset_throttle(self) -> None:
        self._ctrl_data[_THROTTLE_JSON_KEY] = 0

    def turn_left(self) -> None:
        self._ctrl_data[_ANGLE_JSON_KEY] = max(-1., self._ctrl_data[_ANGLE_JSON_KEY] - self._angle_precision)

    def turn_right(self) -> None:
        self._ctrl_data[_ANGLE_JSON_KEY] = min(1., self._ctrl_data[_ANGLE_JSON_KEY] + self._angle_precision)

    def reset_angle(self) -> None:
        self._ctrl_data[_ANGLE_JSON_KEY] = 0

    def use_pilot_mode(self) -> None:
        self._ctrl_data[_MODE_JSON_KEY] = _PILOT_MODE_JSON_VALUE
        self.reset_angle()
        self.reset_throttle()

    def use_user_mode(self) -> None:
        self._ctrl_data[_MODE_JSON_KEY] = _USER_MODE_JSON_VALUE

    def exit(self) -> None:
        self._mqtt_client.loop_stop()
        self._quit_joysticks()
        exit()

    def toggle_help(self) -> None:
        self._display_help = not self._display_help

    def toggle_joystick_mode(self) -> None:
        self._enable_joystick = not self._enable_joystick

    def toggle_record_mode(self) -> None:
        self._ctrl_data[_RECORD_MODE_JSON_KEY] = not self._ctrl_data[_RECORD_MODE_JSON_KEY]

    def _run_ctrl(self, ctrl_key: chr) -> None:
        ctrl_key = ctrl_key.upper()
        _KEY_ACTION.get(ctrl_key, lambda mqtt_driver: ...)(self)
        self._joystick_driving()
        self._mqtt_client.publish(
            topic=self._ctrl_topic,
            payload=dumps(self._ctrl_data)
        )

    def _insert_driving_data(self, img_array: ndarray) -> ndarray:
        if self._display_help:
            return self._add_help_message(img_array)

        img_array = _put_text(img_array, "(Press 'h' for help)", org=(_MARGIN_LEFT, 20), color=(0, 100, 0))

        if self._ctrl_data[_RECORD_MODE_JSON_KEY] is True:
            img_array = _put_text(img_array, "* RECORDING REQUESTED...", org=(_MARGIN_LEFT, 50), color=(0, 0, 200))

        if self._ctrl_data[_MODE_JSON_KEY] == _PILOT_MODE_JSON_VALUE:
            return self._add_auto_pilot_data(img_array)

        return self._add_user_mode_data(img_array)

    @staticmethod
    def _add_help_message(img_array: ndarray) -> ndarray:
        margin_top = 0

        def _add_help_message(msg: str, color: tuple[int, int, int] = (0, 100, 0)) -> None:
            nonlocal img_array
            nonlocal margin_top
            margin_top += 30
            img_array = _put_text(img_array, msg, org=(_MARGIN_LEFT, margin_top), color=color)

        _add_help_message("Press 'h' to return on driving screen")
        _add_help_message("Press 'ESC' to exit the program")
        _add_help_message("Press 'z' or 'top arrow' to increase throttle")
        _add_help_message("Press 's' or 'bottom arrow' to decrease throttle")
        _add_help_message("Press 'SPACE' to reset throttle to zero")
        _add_help_message("Press 'd' or 'right arrow' to increase angle to right side")
        _add_help_message("Press 'q' or 'left arrow' to increase angle to left side")
        _add_help_message("Press 'TAB' to reset angle to zero")
        _add_help_message("Press 'p' to start autopilot mode")
        _add_help_message("Press 'u' to start user mode")
        _add_help_message("Press 'j' to enable/disable joysticks")
        _add_help_message("Press 'r' to enable/disable records for training")

        return img_array

    def _add_auto_pilot_data(self, img_array: ndarray) -> ndarray:
        return _put_text(
            img_array,
            "Driving mode: AUTO PILOT",
            org=(_MARGIN_LEFT, self._frame_size[1] - 10),
            color=(120, 220, 120),
        )

    def _add_user_mode_data(self, img_array: ndarray) -> ndarray:
        img_array = _put_text(
            img_array,
            "Driving mode: USER",
            org=(_MARGIN_LEFT, self._frame_size[1] - 10),
            color=(100, 100, 220),
        )

        throttle: float = self._ctrl_data[_THROTTLE_JSON_KEY]
        img_array = _put_text(
            img_array,
            f"Throttle: {throttle * 100}%",
            org=(_MARGIN_LEFT, self._frame_size[1] - 39),
            color=(20, 20, 220) if throttle > 0 else (220, 20, 20) if throttle < 0 else (0, 0, 0),
        )

        angle: float = self._ctrl_data[_ANGLE_JSON_KEY]
        img_array = _put_text(
            img_array,
            f"Angle: {abs(angle) * 100}% {'RIGHT' if angle > 0 else 'LEFT' if angle < 0 else ''}",
            org=(_MARGIN_LEFT, self._frame_size[1] - 70),
            color=(220, 50, 50) if angle > 0 else (50, 50, 220) if angle < 0 else (50, 50, 50),
        )

        return img_array

    def _init_joysticks(self) -> None:
        pygame.init()
        pygame.joystick.init()
        self._joystick_count: int = pygame.joystick.get_count()
        self._previous_joystick_idx: int = 0
        self._previous_joystick_cmd: tuple[float, float] = (0., 0.)
        self._enable_joystick: bool = True

    @staticmethod
    def _quit_joysticks() -> None:
        pygame.joystick.quit()
        pygame.quit()

    def _joystick_driving(self, joystick_precision: int = 3) -> None:
        if self._enable_joystick is True and self._joystick_count > 0:
            pygame.event.get()
            for i in range(self._joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                try:
                    joystick_cmd: tuple[float, float] = (
                        round(joystick.get_axis(0), joystick_precision),
                        -round(joystick.get_axis(1), joystick_precision)
                    )
                    # Allows secondary joysticks or keyboard to take control when first detected joysticks are not used
                    if (
                            (i == self._previous_joystick_idx and self._previous_joystick_cmd != joystick_cmd)
                            or
                            (i != self._previous_joystick_idx and joystick_cmd != (0., 0.))
                    ):
                        self._ctrl_data[_ANGLE_JSON_KEY] = joystick_cmd[0]
                        self._ctrl_data[_THROTTLE_JSON_KEY] = joystick_cmd[1]
                        self._previous_joystick_cmd = joystick_cmd
                        self._previous_joystick_idx = i
                        return

                finally:
                    joystick.quit()

    def _display_loading_image(self, pending_topic, color=(0, 100, 0)) -> None:
        loading_img: ndarray = np.zeros((self._frame_size[1], self._frame_size[0], 3))
        margin_top: int = 0

        def _add_loading_message(msg: str) -> None:
            nonlocal loading_img
            nonlocal margin_top
            margin_top += 30
            loading_img = _put_text(loading_img, msg, org=(_MARGIN_LEFT, margin_top), color=color, thickness=1)

        _add_loading_message("LOADING...")
        _add_loading_message(f"Waiting for MQTT messages from '{pending_topic}' topic")
        imshow(_IMG_NAME, loading_img)


if __name__ == '__main__':
    DkMqttDriver(
        video_topic='',
        ctrl_topic='',
        username='',
        password='',
        host:'mqtt.diyrobocars.fr',
    )
