
# Dev environment

```
docker build -f .devcontainer/Dockerfile.humble_dev --target gazebo_dev -t humble:dev .
```

## build ws

```
mkdir src
cd src
ros2 pkg create apm_gazebo --build-type ament_cmake 
ros2 pkg create apm_description --build-type ament_cmake 
ros2 pkg create apm_bringup --build-type ament_cmake 
```

## first usage


```bash
#run app
mode GUIDED
arm throttle
takeoff 5
```


# Resource
- [aruco markers](https://github.com/joselusl/aruco_gazebo/tree/master)