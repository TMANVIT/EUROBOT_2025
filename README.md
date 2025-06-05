# EUROBOT 2025

Repository for the [EuroBot 2025 competition](https://www.eurobot.org/eurobot-contest/eurobot-2025/) by MEPhI students.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Build](#build)
  - [Set up Cyclone DDS](#set-up-cyclone-dds)
- [Running](#running)
  - [Launch Robot Stack](#launch-robot-stack)
  - [Launch PC Stack](#launch-pc-stack)
- [To-Do](#to-do)
- [Feedback and Support](#feedback-and-support)

## Prerequisites

- Docker
- Python 3.10+

## Build

The repository supports two launch configurations:
1. For small Mini PC (Raspberry Pi4) on the robot (see [/rpi_launch](./rpi_launch/rpi4_launch.sh))
2. For external PC outside the competition area

### Set up Cyclone DDS

Additional information available on [GitHub](https://github.com/eclipse-cyclonedds/cyclonedds-cxx).

**Important:** Configure [cyclonedds.xml](./cyclonedds.xml) before running.

## Running

### Launch Robot Stack

```bash
make run-pi
```

### Launch PC stack

1. Build docker. Write in terminal 
```
make build-robot
```

2. Select your team configuration:

#### For Blue Team:
```
make run-blue
```

#### For Yellow Team:

```
make run-yellow
```

## To-Do

- Write development blog on Habr
- Migrate to Zenoh
- Separate nodes into different containers
- Release URDF model

## Feedback and Support

For suggestions, improvements, or issue reporting, please contact: 
__Timur Manshin__ at [Telegram](https://t.me/tmanvit).