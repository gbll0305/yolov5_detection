# Requirements

  1. YOLOv5 download: https://github.com/ultralytics/yolov5
  2. pytorch
  3. opencv
  4. v4l2_camera

      ```
      sudo apt update
      sudo apt install ros-${ROS_DISTRO}-v4l2-camera
      ```
  5. (선택사항) v4l2-ctl: change v4l2_camera settings
      ```
      sudo apt install v4l-utils
      
      v4l2-ctl -d /dev/video0 --list-formats-ex   # camera formets 확인
      v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480   # change settings
      ```



# Installation


  ```
  cd workspace
  cd src 
  git clone https://github.com/gbll0305/yolov5_detection.git
  ```


  ### [수정이 필요한 부분] 
  
  /yolo_detection/yolo_detection/yolo_detector.py
  
  1. yolov5 model 가져오는 부분의 경로 수정(Yolo 학습 파일의 위치를 넣어줘야 함)
     
     ```
     # define model path and load the model
     self.declare_parameter('model_path', '/home/chaewon/yolov5/best.pt') #best.pt의 경로를 수정
     model_path = self.get_parameter('model_path').get_parameter_value().string_value
     self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
     ```

     
  2. yolov5의 카메라 정보를 저장하는 부분의 경로 수정(목표물 사진을 저장할 파일 생성 필요)
     
     ```
     # create target_capture folder, which is used to save target images
     self.target_capture_folder = '/home/chaewon/workspace_ros/target_capture' # 저장할 위치 수정
     os.makedirs(self.target_capture_folder, exist_ok=True)
     ```

  ```
  colcon build --symlink-install --packages-select my_bboxes_msg
  colcon build --symlink-install
  source ./install/local_setup.bash
  ```




# Launch

  ```
  # run each nodes saperately
  ros2 run v4l2_camera v4l2_camera_node
  ros2 run yolo_detection yolo_detector
  
  # use launch file
  ros2 run yolo_detection yolo_detedtor.launch.py
  ```



# Camera settings
  Use v4l2-ctl
  
  ```
  # installation
  sudo apt install v4l-utils
  # change camera settings
  v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480
  ```
     

  
