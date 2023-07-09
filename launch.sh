#!/bin/bash

roslaunch_command="roslaunch elephant_robot elephant_robot.launch"

custom_cli_command="find / -name "CLI.py" -type f -exec python3 {} \;"

gnome-terminal --window --title="roslaunch" --command="$roslaunch_command" --window --title="ER CLI" --command="bash -c '$custom_cli_command; exec bash'"
sleep 1

custom_cli_window_id=$(xdotool search --name "ER CLI" | tail -1)
xdotool windowactivate "$custom_cli_window_id"

xdotool windowactivate --sync "$custom_cli_window_id" && xdotool key --clearmodifiers F11
