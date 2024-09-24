# EUROBOT_2025

Files for EuroBot 2025 competition by MEPhI students

## Setup
In this topic should be instruction to use cofig files

## Build/Up

In the root directory make ```docker compose build```, then ```docker compose up```

To allow GUI, in your terminal window printing ```xhost +local:docker``` may be needed

## Run launch file

To start robot scenario in docker container print in terminal window ```ros2 launch .....``` 

## TODO
&#x2610; Fix bugs with running docker compose

&#x2610; Make ROS2 environment

&#x2610; Implement microros-agent node

&#x2610; Start camera node