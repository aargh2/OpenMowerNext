#!/bin/sh
ros2 service call /hardware/clear_emergency std_srvs/srv/Trigger "{}"
