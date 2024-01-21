import Jetson.GPIO as GPIO
import time
from jetson_utils import cudaToNumpy, videoSource
import cv2
import uuid

channel_blocked = 20
channel_free = 21
source = "csi://0"
input = videoSource(source)

def save_img(folder):    
    image = input.Capture()
    array_img = cudaToNumpy(image)
    filename = "data/{}/{}.png".format(folder, uuid.uuid4())
    print(filename)
    print("Saving the image...")
    status = cv2.imwrite(filename, array_img)
    print("Image save: ", status)

def callback_free_img(channel):
    save_img("free")

def callback_blocked_img(channel):
    save_img("blocked")

GPIO.setmode(GPIO.BCM)
GPIO.setup(channel_blocked, GPIO.IN)
GPIO.setup(channel_free, GPIO.IN)

GPIO.add_event_detect(channel_blocked, GPIO.FALLING, callback_blocked_img)
GPIO.add_event_detect(channel_free, GPIO.FALLING, callback_free_img)

while True:
    time.sleep(1)
    print("Watiing...")
    
    