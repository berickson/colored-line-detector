This project contains vision software for a robot entry in the RSSC line color following contest.

See http://www.rssc.org/colorlinefollowing.html for contest information.

The software runs on a JeVois A33 Smart Camera Module which will mounted to a robot during the contest.

For more information on the robot and algorithm, see the write-up on Google Docs at https://docs.google.com/document/d/1tG8bzMibJC4PbiZWjmyg9qU3WrKp2auiUMCJXr3Hj38 .


helpful commands:

sudo ffplay /dev/video1 -pixel_format yuyv422 -video_size 320x240
setpar sensor_algorithm Sensors

setpar sensor_algorithm Edges

setpar sensor_algorithm Both

setpar serout All
setpar serout USB
setmapping 0
streamon

wb
--
# auto
setcam presetwb 1
setcam autoexp 0
setcam autowb 1
setcam autogain 1

# manual
setcam autogain 0
setcam autoexp 1
setcam autowb 0

lab values
-----------
blue  11 54 36
black  4 51 46
red   31 66 50
green  13 49 42


