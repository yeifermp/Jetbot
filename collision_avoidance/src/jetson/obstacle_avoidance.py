import time
import board
import busio
from jetson_utils import cudaToNumpy, videoSource
import jetson_inference

class Wheels:
    stop_cmd: bytes = bytes([0x0])
    forward_cmd: bytes = bytes([0x1])
    reverse_cmd: bytes = bytes([0x2])
    left_cmd: bytes = bytes([0x3])
    right_cmd: bytes = bytes([0x4])
    i2c: busio.I2C
    address = 0x15

    def __init__(self): 
        self.i2c = busio.I2C(board.SCL_1, board.SDA_1)

    def stop(self):
        self.i2c.writeto(self.address, self.stop_cmd)

    def go_forward(self):
        self.i2c.writeto(self.address, self.forward_cmd)

    def reverse(self):
        self.i2c.writeto(self.address, self.reverse_cmd)

    def go_left(self):
        self.i2c.writeto(self.address, self.left_cmd)

    def go_right(self):
        self.i2c.writeto(self.address, self.right_cmd)


wheels = Wheels()
wheels.stop()
source = "csi://0"
input = videoSource(source)

net = jetson_inference.imageNet(model="model/resnet18.onnx", 
                                labels="model/labels.txt",
                                input_blob="input_0", 
                                output_blob="output_0")

while True:
    image = input.Capture()

    if image is not None:
        class_idx, confidence = net.Classify(image)
        class_desc = net.GetClassDesc(class_idx)
        print("Image recognize as {:s}, (Class #{:d}) with {:f} confidence".format(class_desc, class_idx, confidence))

        if class_idx == 1 and confidence > 0.5:
            wheels.go_forward()
        else:
            wheels.go_left()
            time.sleep(0.5)
