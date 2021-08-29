# MEAM520: Introduction to Robotics
### Date Created: 08/26/2021

Maintainers: Contact FY2021 TAs

# Client Directory

This directory houses all the python code you will use and implement to control the robot in simulation and real life.

# Subdirectory Structure:
- `core`: contains support code we provide to you, such as helper functions and interfaces for interacting with the robot (either simulated or real!)
- `demos`: contains some example scripts, mostly for the instruction team
- `lib`: will contain functions implementing algorithms to solve computational problems relating to the class material, such as forward and inverse kinematics and trajectory planning
- `labs`: will contain test scripts that use the algorithms you implement in `lib` and the tools we provide in `core` to control the robot and achieve tasks

`lib` and `labs` will be the main places you are working this semester!


# Native Install Instructions (NOT REQUIRED FOR VIRTUAL MACHINE)
---
**NOTE**

These installation instructions are for the TAs when setting up the Virtual Machine, and can also be followed by experienced users to set up the lab environment on a native Ubuntu Linux installation. These steps do not need to be followed by students using the Virtual Machine. For all other users, skip to the section on forking this repo.

---

## panda_simulator installation

To get started using this development environment, you must first follow the instructions to install [panda_simulator](https://github.com/justagist/panda_simulator), a Gazebo-based simulator for the Franka Emika Panda robot. The only difference is that you must name the catkin workspace `meam520_ws` to avoid conflicts with other projects.

The instructions specify to use `catkin build`, but we recommend buildint with `catkin_make_isolated` instead.

Once you've completed that installation, add

```
source ~/meam520_ws/devel_isolated/setup.bash
```
to your `~/.bashrc` to source the workspace. If all has been done correctly, you should be able to run

```
roslaunch panda_gazebo panda_world.launch
```

to launch the Gazebo simulation.

## meam520_labs installation

Now, clone this repo:

```
cd ~/meam520_ws/src
git clone https://github.com/MEAM520/meam520_labs
```

and run `catkin_make_isolated` again.

Once the build finishes, open a new terminal and run

```
roslaunch meam520_labs lab0.launch
```

To check that the installation is successful, run a demo script:

```
cd ~/meam520_ws/src/meam520_labs/client/demos
python position_demo.py
```

You should see the robot in Gazebo moving back and forth.


# Forking This Repo

- git is a code versioning tool, and can be a powerful resource.
- This year we will use git in a limited fashion to ease the dissemination of code.
- So that each student can have their own individual work we are going to have you work on a fork of the repository in your own gitHub account.

### Setting up your gitHub account
- XXX Torrie needs to figure this out

### Forking an existing git repository:
- A fork is a copy of a git repository which makes a deep copy of a code base and makes it possible for new development.
- We are going to have you use the following steps to fork the MEAM520_fall2021 code base.

XXX Torrie Needs to check these (forking is disbaled for private repos)

1. Login to your GitHub account
2. Go to: https://github.com/MEAM520/meam520_fall2021
3. At the top right corner you will see Fork. Click on this, and you will be prompted to fork the repository to your account
4. Go to the top right corner and select your repositories, copy the repository code link
5. Open a terminal:

```
$ cd meam520/catkin_ws/scripts
$ git clone URL
$ git reomte -v
$ git remote add upstream https://github.com/MEAM520/meam520_fall2021.git
$ git remote -v

```

* Note the git remote -v will show you where your remote repository is pointing. (THIS NEEDS CLARITY)
* It is important to see that your origin points to your forked repository in your account and that the upstream is pointing to the main repo
* From here you will need to perodically perform a git pull upstream which will update your forked repository with updates that we make in the code

## Using git for version control:
- It is not necessary for this semester for you to use git in any other way than to occassionally pull new code from the upstream repository
- However, if you want to practice git and keeping track of your own personal forked repo here are places to start

XXX Torrie finish this

XXX Torrie add further resources (plus point to talk on canvas)
