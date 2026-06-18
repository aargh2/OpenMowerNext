#!/bin/sh
ros2 topic pub --once /hardware/ui_event open_mower_next/msg/UiButtonEvent "{button_id: 3, press_duration: 0}"
