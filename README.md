# Lidar - Camera Calibration

![](./img/diagram.png)

## Run through Docker

1. Clone the repository.
2. Execute (inside the folder):

```
docker build .
```

3. Run the image (with X11 support):
```
docker run --gpus all -it --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <repository location>:/lidar_camera_calibration <image number>
```
Inside the container:
```
cd /root/catkin_ws
ln -sf /lidar_camera_calibration/ src/
catkin_make
source devel/setup.bash 
```


Notes:
To enable the GUI do not forget to run this on a local terminal.
```
xhost +
```
Also, use this docker run command to share the same roscore between the docker container and the host.
```
docker run --gpus all -it --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <repository location>:/lidar_camera_calibration <image number>
```

## Prepare data for calibration

There are two topics that are necessary for the calibration process. One for video and one for Lidar.
In addition, you will need the intrinsic parameters of the camera.
Then set the input topics at the launch file that you will execute.

## Geometric calibration

Run the bagfile (or publish the necessary topics), execute:

```
roslaunch lidar_camera_calibration geometric.launch
```

and set the distance threshold and the number of consequent parameters.

## Temporal calibration

Run the bagfile (or publish the necessary topics), execute:

```
roslaunch lidar_camera_calibration temporal.launch
```

and set the distance threshold and the number of consequent parameters.
