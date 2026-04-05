# ROS2
> Learning ROS2 from basics to advanced

## Using Docker image ROS2 jazzy

### Starting up (beginning the session)

- Enable Display Forwarding (Need if rebooted the PC)

```
xhost +local:root
```

### Turn on the Docker Container

```
sudo docker start ros2_jazzy
```

### Enable the Container

```
sudo docker exec -it ros2_jazzy bash
```

### Working inside the Container
Every time enter the container or open a new terminal 

- Activate ROS2

```
source /opt/ros/jazzy/setup.bash
```

- Navigate to workspace

```
cd /root/ros2_ws
```

### Open new terminal or tab

- Enable Container

```
sudo docker exec -it ros2_jazzy bash
```
- Activate ROS

```
source /opt/ros/jazzy/setup.bash
```

### Closeing Down

```
exit
```

### Turn off the Container

```
sudo docker stop ros2_jazzy
```
