#!/usr/bin/env python3.8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


net = cv2.dnn.readNetFromONNX("yolov5n.onnx")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

file = open("items.txt", "r")
classes = file.read().split("\n")

class Detection:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.detection_pub = rospy.Publisher("detection_info", String, queue_size=10)

    def callback(self, data):

        detection_info=""

        try:
            cv_image=np.fromstring(data.data, np.uint8)
            cv_image_2=cv2.imdecode(cv_image, cv2.IMREAD_COLOR)
            height, width, channels = cv_image_2.shape
            height=int(height*1.5)
            width=int(width*1.5)
            image=cv2.resize(cv_image_2, (width,height))
        except:
            return
        
        blob = cv2.dnn.blobFromImage(image, scalefactor=1/255, size=(640, 640), mean=[0, 0, 0], swapRB=True, crop=False)
        net.setInput(blob)
        detections = net.forward()[0]

        classes_id = []
        confidences = []
        boxes = []

        img_width, img_height = image.shape[1], image.shape[0]
        x_scale, y_scale = img_width / 640, img_height / 640

        for i, row in enumerate(detections):
            confidence = row[4]
            if confidence > 0.2:
                classes_score = row[5:]
                ind = np.argmax(classes_score)
                if classes_score[ind] > 0.5:
                    classes_id.append(ind)
                    confidences.append(confidence)
                    cx, cy, w, h = row[:4]
                    x1 = int((cx - w / 2) * x_scale)
                    y1 = int((cy - h / 2) * y_scale)
                    width = int(w * x_scale)
                    height = int(h * y_scale)
                    box = np.array([x1, y1, width, height])
                    boxes.append(box)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.5)

        for i in indices:
            x1, y1, w, h = boxes[i]
            label = classes[classes_id[i]]
            conf = confidences[i]
            text = f"{label} {conf:.2f}"
            if label=="car" or label=="bus" or label=="aeroplane":
                detection_info=f"{int(x1+(w/2))} {int(y1+(h/2))} {h}"
            cv2.rectangle(image, (x1, y1), (x1 + w, y1 + h), (255, 0, 0), 2)
            cv2.putText(image, text, (x1, y1 - 2), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 255), 2)

        cv2.imshow("camera", image)
        cv2.waitKey(3)

        if len(detection_info)!=0:
            self.detection_pub.publish(detection_info)


if __name__ == '__main__':
    rospy.init_node('camera', anonymous=False)
    detection = Detection()
	
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()
