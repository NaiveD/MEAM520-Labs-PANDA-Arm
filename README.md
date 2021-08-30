# MEAM520: Introduction to Robotics
### Date Created: 08/26/2021

Maintainers: Contact FY2021 TAs

# Subdirectory Structure:
- `core`: contains support code we provide to you, such as helper functions and interfaces for interacting with the robot (either simulated or real!)
- `lib`: will contain functions implementing algorithms to solve computational problems relating to the class material, such as forward and inverse kinematics and trajectory planning
- `labs`: will contain test scripts that use the algorithms you implement in `lib` and the tools we provide in `core` to control the robot and achieve tasks
- `ros`: contains code necessary to launch the simulator. You won't need to work in this directory.

`lib` and `labs` will be the main places you are working this semester!


# Native Install Instructions (NOT REQUIRED FOR VIRTUAL MACHINE)
---
**NOTE**

These installation instructions are for the TAs when setting up the Virtual Machine, and can also be followed by experienced users to set up the lab environment on a native Ubuntu Linux installation. These steps do not need to be followed by students using the Virtual Machine. For all other users, skip to the section on forking this repo.

---

## Operating System

The simulator must be run on Ubuntu 20.04 with ROS noetic installed. You can follow the standard installation instructions for [Ubuntu 20.04](https://phoenixnap.com/kb/install-ubuntu-20-04) and [ROS noetic](http://wiki.ros.org/noetic/Installation).

## panda_simulator installation

To get started using this development environment, you must first follow the instructions to install [panda_simulator](https://github.com/justagist/panda_simulator), a Gazebo-based simulator for the Franka Emika Panda robot. The only difference is that you will name the catkin workspace `meam520_ws` to avoid conflicts with other projects.

The instructions specify to use `catkin build`, but we recommend building with `catkin_make_isolated` instead.

Once you've completed that installation, add

```
source ~/meam520_ws/devel_isolated/setup.bash
```

to your `~/.bashrc` to source the workspace. If all has been done correctly, you should be able to run

```
roslaunch panda_gazebo panda_world.launch
```

to launch the Gazebo simulation.

### Speed up Gazebo shutdown

Since you may be launching and killing Gazebo many times this semester, we recommend reducing the clean shutdown wait time for a better experience. Edit the file:
```
cd /opt/ros/noetic/lib/python3/dist-packages/roslaunch
sudo vim nodeprocess.py
```
and change the lines that say
```
...
_TIMEOUT_SIGINT = 15.0 #seconds

_TIMEOUT_SIGTERM = 2.0 #seconds
...
```
to `2.0` and `1.0` respectively.

## Forking This Repo
git is a code versioning tool, and can be a powerful resource. This year we will use git in a limited fashion to ease the dissemination of code. So that each student can have their own individual work we are going to have you work on a fork of the repository in your own gitHub account.

If you have a GitHub account you can skip the next step and move onto forking an existing git repository section.

### Setting up a gitHub account
- Go to: https://github.com
- Select signup
- follow the prompts to make an account using your upenn email.
- login into your account.

### Fork this repository

Follow the instructions in the Lab0 handout PDF to set up a new fork of this repo. Note: since you are REQUIRED by Penn's code of Academic Integrity to keep your solutions in a Private repo, you will need to take some additional steps which are explained in the handout.

<!-- ### Forking an existing git repository:
- A fork is a copy of an existing git repository. The copy is a deep copy of a code base and makes it possible to significantly diverge from the current direction of the existing repo.
- We are going to have you use the following steps to fork the MEAM520_fall2021 code base, so that each student has their own code space to develop in and can get consistent updates from the TAs.

1. If you are not already, login to your GitHub account
2. Go to: https://github.com/MEAM520/meam520_fall2021
3. At the top right corner you will see Fork. Click on this, you will be prompted to fork the repository to your account and redirected to the forked repository once it is created.
4. Go to the top right corner and select your repositories, copy the repository code link
5. Clone the newly forked repo (the link is found in the top right corner in the green code box), open a terminal:

```
cd ~/meam520_ws/src
git clone https://github.com/<YOUR_USERNAME>/meam520_sim.git
```

Now we are going to make it possible for you to get updates from the TAs main repository.
First it is important to understand that git is a tool which is used locally to keep a history of your code.
To ensure that code is backed up to an additional location outside of your computer a remote is setup.
GitHub is an example of a location which stores remote git repositories and acts as a way to backup code to a secondary location.

To see that sure your local git repository is setup correctly type the following command:
```
$ cd meam520_sim
$ git reomte -v
```

You should see:
```
> origin  https://github.com/YOUR_USERNAME/meam520_sim.git (fetch)
> origin  https://github.com/YOUR_USERNAME/meam520_sim.git (push)
```
Origin is the primary remote location, and this is pointing to the repository you forked.

Now we are going to add an additional pointer telling your git repository to check updates made at the original forked location (in this case where the TAs will make updates and release new projects).
In git language this is called setting the remote upstream.

Do the following:
```
$ git remote add upstream https://github.com/MEAM520/meam520_labs.git
$ git remote -v
```

You should now see:

```
> origin    https://github.com/YOUR_USERNAME/meam520_sim.git (fetch)
> origin    https://github.com/YOUR_USERNAME/meam520_sim.git (push)
> upstream  https://github.com/MEAM520/meam520_labs.git (fetch)
> upstream  https://github.com/MEAM520/meam520_labs.git (push)
```

Notice that origin still points to your fork, and that upstream is now pointing to the repository that is maintained by the TAs.
Now that an upstream is set we will ask that you periodically use the following command:

```
git pull upstream master
```

This will ensure that you get updates when TAs make changes. We will add reminders any time we update the code for you to do this.
 -->

## meam520_labs installation

Once you have a forked, cloned, and set the upstream for the git repository run `catkin_make_isolated` again.

Once the build finishes, open a new terminal and run:

```
roslaunch meam520_labs lab0.launch
```

To check that the installation is successful, run a demo script:

```
cd ~/meam520_ws/src/meam520_labs/labs/lab0
python demo.py
```

You should see the robot in Gazebo moving.

# Additional Resources:
## Using git for version control:
- It is not necessary for this semester to use git in any other way than to occasionally pull new code from the upstream repository. However, if you want to practice git and keeping track of your own personal forked repo there are basic commands to get started with.
- As a version system git keeps track of changes. If you have made changes to code you can do the following:

```
$ git status # Red changed files will appear
$ git add . # This adds all the files you have changed
$ git status # The red files will become green
$ git commit -m "Write a commit message" # This is where you tell your local git repo that you have made changes and are tagging these changes with this git commit message
$ git push # This pushes code to gitHub, your remote backup
```

XXX Torrie add further resources (plus point to talk on canvas)
