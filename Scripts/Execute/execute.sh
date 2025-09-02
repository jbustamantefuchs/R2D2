#!/bin/bash

gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/actions/actions.py; exec bash"
gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/drive/get_n_send.py; exec bash"
gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/drive/nr_control.py; exec bash"
gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/drive/joystick_drive.py; exec bash"
gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/drive/send_arduino_move.py; exec bash"
gnome-terminal -- bash -c "python3 ~/jose/r2d2/python/llava/no_hey_r2.py; exec bash"

