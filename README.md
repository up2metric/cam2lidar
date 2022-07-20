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
docker run --gpus all -it --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <repository location>:/cam2lidar <image number>
```
Inside the container:
```
cd /root/catkin_ws
ln -sf /cam2lidar/ src/
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
docker run --gpus all -it --privileged --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v <repository location>:/cam2lidar <image number>
```

## Prepare data for calibration
![](./img/tag36h11.jpg)

There are two topics that are necessary for the calibration process. One for video and one for Lidar.
In addition, you will need the intrinsic parameters of the camera.
Then set the input topics at the launch file that you will execute.

## Geometric calibration

Run the bagfile (or publish the necessary topics), execute:

```
roslaunch cam2lidar geometric.launch
```

and set the following parameters in the config folder.

```
# Geometric calibration
reproj_error: 8
intensity_thres: 200
distance_from_prev: 100
horizontal_dimension: 3840
vertical_dimension: 2160
grid_horizontal_division: 5
grid_vertical_division: 5
```

**reproj_error**: Reprojection error of PnP

**intensity_thres**: Lidar intensity threshold that is considered to be coming from the reflective tape

**distance_from_prev**: Distance (in px) from previous apriltag in order for the movement to be considered as static

**horizontal_dimension/vertical_dimension**: Dimensions of the image

**grid_horizontal_division/grid_vertical_division**: Shape of grid, in order to have one measurement per rectangle

## Temporal calibration

Run the bagfile (or publish the necessary topics), execute:

```
roslaunch cam2lidar temporal.launch
```

and set the parameters as mentioned in the Geometric calibration section.
