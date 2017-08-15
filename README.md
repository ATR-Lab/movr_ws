# movr_ws
> ROS workspace for movr

# Table of contents

## Dependencies

### ROS

```
# rosserial
$ sudo apt install ros-kinetic-rosserial
$ sudo apt install ros-kinetic-rosserial-arduino 
```

```
# Joy
$ sudo apt install ros-kinetic-joy

# Set permission to make accessible to ROS
$ ls -l /dev/input/jsX              # X is the joystick number
$ sudo chmod a+rw /dev/inputjsX
```

```
# Openni - Kinect
$ sudo apt install ros-kinetic-openni-launch
```

```
# CV-Bridge
$ sudo apt install ros-kinetic-cv-bridge
```

### Python

```
# API.AI
# Chat bot API
# https://github.com/api-ai/apiai-python-client
$ pip install apiai
```

```
# SpeechRecognition
# Google speech recognition API wrapper
# https://pypi.python.org/pypi/SpeechRecognition/
$ pip install SpeechRecognition
```

```
# gTTS 1.2.1
# Google text-to-speech API wrapper
# https://pypi.python.org/pypi/gTTS/1.2.1
$ pip install gTTS
```

```
# pocketsphinx-python
# CMU speech recognition
# https://github.com/cmusphinx/pocketsphinx-python
$ pip install pocketsphinx
```

```
# pygame
# Game engine. Required to use audio library
# https://www.pygame.org/wiki/GettingStarted
$ pip install pygame
```

```
# json
# JSON library
$ pip install json
```
## How to use
```
# Run roscore
$ roscore
```



## Development

## References
