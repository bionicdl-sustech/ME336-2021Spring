# end2end <!-- omit in toc -->

The folder contains end-to-end robot learning methods by integrating some state-of-the-art researches in robot learning.

As the running environment for each method is different, DeepClaw adopts concepts from cloud robotics. We put the running environments for end-to-end methods which requires heavy computations in a docker container on the server, and deploy the robot control and basic computations on a user computer.

Currently we have two servers: Goldenboy and Serbreeze. Currently they are running Ubuntu16.04 and cuda9.0. We plan to upgrade them to Ubuntu18.04 and cuda10 soon.

|           | Goldenby                                     | Serbreeze |
|-----------|----------------------------------------------|-----------|
| Memory    | 251.8G                                       |           |
| Processor | Intel® Xeon(R) CPU E5-2698 v4 @ 2.20GHz × 40 |           |
| GPU       | Tesla V100 32G x4                            |           |
| Storage   | 7.6T                                         |           |
| Users     | Standard: user-1, user-2, user-3, user-4     |           |
| IP        | 10.20.123.35                                 |           |

**List of algorithms:** <!-- omit in toc -->

- [SSD: segmentation + recogniction](#SSD)
- [Detectron2: segmentation + recogniction](#Detectron2)
- [RLBench: Simulation environment for robot learning](#RLBench)

## SSD:
Please refer to the folder [SSD](SSD) for further details.

## Detectron2:
Detectron2 is Facebook AI Research's next generation software system that implements state-of-the-art object detection algorithms.

All the codes are integrated in DeepClaw. As DeepClaw does not unify installation environment for end2end learning modules, users are suggested to follow the installation guidelines of the original codes and run the module in a docker environment.

Run corresponding docker image (pytorch 1.5.0) on Goldenboy:
nvidia-docker run --rm -it -v /raid:/raid pytorch_py3:v20.03 bash

Please refer to the folder [detectron2](detectron2) for further details.

## RLBench
RLBench is an ambitious large-scale benchmark and learning environment designed to facilitate research in a number of vision-guided manipulation research areas, including: reinforcement learning, imitation learning, multi-task learning, geometric computer vision, and in particular, few-shot learning.

DeepClaw does not contain the original code of the project as it is fairly large in size. Please git clone the latest codes from the [project's website](https://github.com/stepjam/RLBench.git) or clone a [forked version](https://github.com/ancorasir/RLBench.git) from our website.

The users are suggested to install the codes in a virtual environment such as anaconda env.
1. Install the latest CoppeliaSim (V4.0.0): Downloads and extract the files. Add the following to your ~/.bashrc file
```bash
export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```


2. Install PyRep
```bash
git clone https://github.com/stepjam/PyRep.git
cd PyRep
pip3 install -r requirements.txt
python3 setup.py install --user
```
   - You  might encounter the qt version problem. The CoppeliaSim uses its own qt library in its root folder. In case your system find other qt installations for example under anaconda, you need to uninstall that qt library. Refer to [this issue](https://github.com/stepjam/PyRep/issues/76)

3. Install RLBench
```bash
pip3 install -r requirements.txt
python3 setup.py install --user
```
