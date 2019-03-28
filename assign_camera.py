import os
import glob
import time
time.sleep(5)

path = '/tmp/caminfo/'
os.system("ifdown eth0")
os.system("ifup eth0")
os.system("sudo ./jetson_clocks.sh")

#clearing camera bandwidth
os.system("sudo rmmod uvcvideo")
os.system("sudo modprobe uvcvideo quirks=128")

#setting resolution
os.system("v4l2-ctl -d /dev/video0 --set-fmt-video=width=320,height=240")
os.system("v4l2-ctl -d /dev/video1 --set-fmt-video=width=320,height=240")
os.system("v4l2-ctl -d /dev/video2 --set-fmt-video=width=320,height=240")

#getting camera info
os.system("v4l2-ctl -d /dev/video0 --info > /tmp/caminfo/video0.txt")
os.system("v4l2-ctl -d /dev/video1 --info > /tmp/caminfo/video1.txt")
os.system("v4l2-ctl -d /dev/video2 --info > /tmp/caminfo/video2.txt")

files = os.listdir(path)
#assigning camera id
for file in files:
    filename, file_extension = os.path.splitext(file)
    searchfile = open(path +file, "r")
    for line in searchfile:
        if "LifeCam" in line:
            LIFECAM = "/dev/" +filename
            print(filename)

    searchfile.close()

cameras = glob.glob("/dev/video*")
print(cameras)
cameras.remove(LIFECAM)
print(cameras)
if LIFECAM is not None:
   #configuring lifecam settings
   os.system("v4l2-ctl -d " + LIFECAM + " -c exposure_auto=1 -c exposure_absolute=1")
   os.system("v4l2-ctl -d " + LIFECAM + " -c white_balance_temperature_auto=0")
   os.system("/home/nvidia/Desktop/TapeAgain/tapeDetection " + LIFECAM + "&") #+ " > /var/log/tapedetect.txt 2>&1")

#os.system(
#    "gst-launch-1.0 v4l2src device=/dev/" + LIFECAM +" ! nvvidconv flip-method=0 \
#    ! 'video/x-raw(memory:NVMM),width=320,height=240' ! \
#    omxh264enc control-rate=2 bitrate=1000000 ! 'video/x-h264, stream-format=(string)byte-stream'  ! \
#    h264parse ! rtph264pay mtu=1400 ! udpsink host=10.18.68.5 port=5800 sync=false async=false &")
    
#os.system("gst-launch-1.0 v4l2src device=" + cameras[1] +" ! nvvidconv flip-method=0 \
#    ! 'video/x-raw(memory:NVMM),width=320,height=240' ! \
#    omxh264enc control-rate=2 bitrate=1000000 ! 'video/x-h264, stream-format=(string)byte-stream'  ! \
#    h264parse ! rtph264pay mtu=1400 ! udpsink host=10.18.68.5 port=5801 sync=false async=false > backlog.txt &")
port = 1181#5801
for cam in cameras: 
   os.system("gst-launch-1.0 v4l2src device=" + cam +" ! nvvidconv flip-method=0 ! 'video/x-raw(memory:NVMM),width=320,height=240' ! omxh264enc control-rate=2 bitrate=1000000 ! 'video/x-h264, stream-format=(string)byte-stream'  ! h264parse ! rtph264pay mtu=1400 ! udpsink host=10.18.68.5 port=" +str(port) +" sync=false async=false &")
   port += 1
