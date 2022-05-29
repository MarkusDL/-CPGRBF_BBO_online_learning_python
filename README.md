# Online generic neural locomotion control framework

__The framework is based on neural control and black-box optimization. The neural control combines a central pattern generator (CPG) and a radial basis function (RBF) network into a CPG-RBF network. The control network acts as a neural basis that can produce arbitrary rhythmic trajectories for the joints of robots.__

- [Install](#install)
- [Getting Started](#getting-started)

## Install

First, we need to set up the v-rep sim. Typically the simulator is installed in your home directory - but it can be anywhere.
1. Download the latest version of V-REP (aka CoppeliaSim) [from the downloads page](http://www.coppeliarobotics.com/downloads.html)
2. Extract the downloaded .zip file in the V-REP directory.
3. Go to the extracted directory - should be called `VREP1`
5. copy and place in /utils/simulationID.txt in the VREP directory.

Now we need to install the required python libraries:

```bash
sudo apt install python3-pip
pip3 install -r requirements.txt
```

The neural controllers use ROS to communicate with v-rep. So make sure that you have `ros-xxx-desktop-full` installed ([Install ROS](http://wiki.ros.org/ROS/Installation)), and folow the VREP/Coppeliasim Ros turtorial ([Ros tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm))

You should be good to go!

#### Troubleshooting

Below are some problems you may encounter during installation. If none of these solves your problem, please raise an issue.
- error: command `x86_64-linux-gnu-gcc` failed
  - You may be missing packages needed for building python extensions. Try: `sudo apt-get install python3-dev`, and then re-run the installation.

## Getting Started

1. First, take a look at the V-REP [tutorials](http://www.coppeliarobotics.com/helpFiles/en/tutorials.htm).
2. Start your simulation
```bash
cd {VREP_WORKER_ROOT}/VREP1/
./vrep.sh FRAMWORK_PATH/CPGRBFN_online_learning_python/simulations/MORF_open_world.ttt
```
or
```bash
./coppeliaSim.sh FRAMWORK_PATH/CPGRBFN_online_learning_python/simulations/MORF_open_world.ttt
```

4. Go to the pythonscripts directory.
```bash
cd FRAMWORK_PATH/CPGRBFN_BBO_online_learning_python/pythons_cripts/
```
5. run online controller from gait memory, if empty gait memory is used, create new folder for it and the controplelrs will be learned from scratch

```bash
python3 MultiLearning.py RBFN direct false false false 50 7 4 0.1 0.04 0.995 ../data/gait_memories/{The chosen gait memory to run}/
```
Help for the arguments used for the python call can be found with:
```bash
python3 main.py -h
```

6. If a single learning test is needed run the following command:

```bash
python3 Learning.py RBFN direct false false false 50 7 4 0.1 0.04 0.995 ../data/{datadir}/
```
