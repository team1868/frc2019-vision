# frc2019-vision
# Team 1868 FRC 2019 Deep Space Vision Repository #
### Projects ###
* Hatch Target Detection
* Cargo Detection (HSV)

### Dependencies ###
* Python 3
* NumPy
* OpenCV 3

### Startup Notes ###
The startup script is in /etc/init.d/ and it runs the assign camera code, which is in the home directory. Currently streams up to 2 webcam cameras around 3.4MB using gstreamer and runs the tape detection code with Microsoft Lifecam 360. 
