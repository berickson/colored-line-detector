This project contains vision software for a robot entry in the RSSC line color following contest.

See http://www.rssc.org/colorlinefollowing.html for contest information.

The software runs on a JeVois A33 Smart Camera Module which will mounted to a robot during the contest.

For more information on the robot and algorithm, see the write-up on Google Docs at https://docs.google.com/document/d/1tG8bzMibJC4PbiZWjmyg9qU3WrKp2auiUMCJXr3Hj38 .


helpful commands:

sudo ffplay /dev/video1 -pixel_format yuyv422 -video_size 320x240
setpar sensor_algorithm Sensors

setpar sensor_algorithm Edges

setpar sensor_algorithm Both

serout All
serout Usb
setmapping 0
streamon