# Overview
This repo is meant to be an all inclusive logging + info thing for making the RHD2164 do actual work, it is currently broken into 5 major folders

### info
On the tin, contains more info related to the project, pictures, pdfs, write-ups etc.

### firmware
Contains all the RP235XX code to talk with the RHD and act as a webserver for it

### apps
This holds any apps written for basic tests or logging utilities, most likely some python scripts managed by uv

### message_defs
The flatbuffer message defs to interact with the firmware, used by the firmware with flatcc and any apps with regular flatc

### hardware
A KiCad project for a daughter board to a pico 2w for the LVDS driver and battery managment 

# Oh btw
The root of this repo is also an obsidian vault, and that is the intended way to view this README and any of the other .md files.
