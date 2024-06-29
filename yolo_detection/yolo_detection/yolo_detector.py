import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_bboxes_msg.msg import YoloObstacle, YoloTarget
import cv2
import torch
from cv_bridge import CvBridge
import time

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        
        
        # define model path and load the model
        #/home/gracekim/yolov5/yolov5/runs/train/exp3/weights/best.pt
        self.declare_parameter('model_path', '/home/chaewon/yolov5/best.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
        
        '''
        #normal model <- CHANGE IF DIRECTORY IS CHANGED
        #self.model = YOLOv5Model('/home/chaewon/workspace_ros/yolov5/yolov5s.pt')
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        '''
        
        # create publishers
        self.publisher_obstacle = self.create_publisher(YoloObstacle, 'yolo_obstacle', 10)
        self.publisher_target = self.create_publisher(YoloTarget, 'yolo_target', 10)
        
        # create cv_bridge instance
        self.bridge = CvBridge()
        # start the webcam. and if it is not opened, print error message
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            return

        # create target_capture folder, which is used to save target images
        self.target_capture_folder = '/home/chaewon/workspace_ros/target_capture'
        os.makedirs(self.target_capture_folder, exist_ok=True)
        
        # timer for publishing target image
        self.timer_period = 5.0  # seconds
        self.last_capture_time = time.time()
        
        self.process_camera_frames()

    def process_camera_frames(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("프레임을 읽을 수 없습니다.")
                break
            
            # send frame to YOLOv5 model
            results = self.model(frame)
            # extract bounding box, labels
            labels, cords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

            # draw bounding box in frame & publish obstacle message
            for i in range(len(labels)):
                row = cords[i]
                if row[4] >= 0.3:
                    x1, y1, x2, y2 = int(row[0] * frame.shape[1]), int(row[1] * frame.shape[0]), int(row[2] * frame.shape[1]), int(row[3] * frame.shape[0])
                    label = self.model.names[int(labels[i])]
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    
                    # publish obstacle message
                    obstacle_msg = YoloObstacle()
                    obstacle_msg.label = label
                    obstacle_msg.x = x_center
                    obstacle_msg.y = y_center
                    self.publisher_obstacle.publish(obstacle_msg)
                    
                    # draw bounding box and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'{label} {row[4]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            
            # check if it's time to publish target image
            current_time = time.time()
            if current_time - self.last_capture_time >= self.timer_period:
                self.publish_target_image(frame)
                self.last_capture_time = current_time
            

            # display frame
            cv2.imshow('YOLOv5 Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()


    def publish_target_image(self, frame):
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        target_msg = YoloTarget()
        target_msg.image = image_msg
        
        timestamp = int(time.time())
        file_path = os.path.join(self.target_capture_folder, f'target_{timestamp}.jpg')
        cv2.imwrite(file_path, frame)
        
        self.publisher_target.publish(target_msg)
        self.get_logger().info(f"Target image published and saved to {file_path}")




def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
