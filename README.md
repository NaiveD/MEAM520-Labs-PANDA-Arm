# MEAM520: Introduction to Robotics
### Date Created: 08/26/2021

Maintainers: Contact FY2021 TAs

# Directory Structure:
- lib: Will contain functions that you will implement that relate to material learned in class
- labs: This is where your test code will exist
- panda_robot: XXX Jake is working on this

# How this repository should be used:
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
