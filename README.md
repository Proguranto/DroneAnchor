# 🚁 PaparazzoDrone 🚁
PaparaazzoDrone is a single-object tracking application for the DJI Tello drone. It takes the images from the DJI Tello's video camera, detects a person from the image, and then centers its camera view to the person while staying within 200-300 cm away. The application uses the YOLOv5 model for object detection and uses the simple-pid python module for drone PID controls.


By: Grant Tannert and Tim Mandzyuk

### Paper reference: [PapparazzoDrone Report](./PaparazzoDrone_report.pdf)

## 😎 Goals and Motivation 😎
Inspired by the camera man running in track events, we thought it would be cool a fun project to have a drone simulate its behavior. The name is motivated from paparazzo trying to focus and follow on celebrities where the drone is the paparazzo and the tracked person is the celebrity.  
 <img src="https://media.tenor.com/yhMhGozOVk4AAAAM/world-photography-day-funny-photographer.gif" width="200">

## Demos
Here is a demo of the application from the drone's perspective:  
<img src="./demo/drone_view_follow.gif" width="400">  

Here is a demo of the application from the a third person's perspective:  
<img src="./demo/drone_follow.gif" width="300" height="500">   

## Credits
The YOLOv5 model: <a href="https://github.com/ultralytics/yolov5">https://github.com/ultralytics/yolov5</a>  
The simple-pid module: <a href="https://pypi.org/project/simple-pid/">https://pypi.org/project/simple-pid/</a>