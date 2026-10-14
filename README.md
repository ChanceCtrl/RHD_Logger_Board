# Overview
This repo is meant to be an all inclusive logging + info thing for making the RHD2164 do actual work, it is currently broken into 3 major folders

### info
On the tin, contains more info related to the project, pictures, pdfs, write-ups etc.

### vhdl
This folder contains all the vivado/VHDL related stuff for getting a working interface for talking with the [[rhd2164]] to the [[python-app]].

### python-app
A UV managed python app (who would've guessed) that is meant to handle logging info from the RHD2164 into whatever format.

# Oh btw
The root of this repo is also an obsidian vault, and that is the intended way to view this README and any of the other .md files.

# TODO
- [ ] RHD DDR receive logic
- [ ] RHD transmit logic
- [ ] RHD Register system in VHDL
- [ ] Basic python app
- [ ] Try a real-time capture