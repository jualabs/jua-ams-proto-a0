from picamera import PiCamera
from time import sleep
import os

camera = PiCamera()

camera.rotation = 90
camera.resolution = (320, 180)
camera.framerate = 24
i = 1
s = os.statvfs('/')
free_mem = (s.f_bavail * s.f_frsize) / 1024
# camera.annotate_text = "Pedro estrogonificamente"
# camera.start_preview()
camera.start_recording('1.h264')
camera.wait_recording(5)
while(free_mem > 500000):
	camera.split_recording('%d.h264' % i)
	camera.wait_recording(60*10)
	s = os.statvfs('/')
	free_mem = (s.f_bavail * s.f_frsize) / 1024
        print(free_mem)
	i = i + 1
camera.stop_recording()
# camera.stop_preview()
