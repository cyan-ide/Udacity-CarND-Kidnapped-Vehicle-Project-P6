# **Kidnapped Vehicle** 

## Udacity Project 6 Writeup

---

**Kidnapped Vehicle Project**

The goals / steps of this project are the following:
* Implement vehicle localization using particle filter
* Assure accuracy and execution time requirements (as verified by simulator)

---

**Summary:** Similar as Kalman Filters this project was about filling out existing project template with missing parts of the algorithm. The key challenge was in sticking all pieces of code introduced during the class together and assuring that all calculations are done bug free and in proper order. As most numerical methods, Particle Filter easily fails due even small miscalcilations, issues with variable precision etc. (e.g. common problem was weights quickly converging to zero due to too big differences of particle landmark observations vs. real landmark positions - located in the exponent of the multi-variate distribution at the end of the weight update step). Among more trivial issues, turns out the simulator itself is quite resource hungry, therefore running it on the same machine as the Particle Filter algorithm can easily cause the algorithm to fail execution time requirement - I found this a bit troublesome and biased solution for testing algorithm performance (I found out that on MacOS one way to mitigate this issue to certain extent is simply: 1) decreasing simulator resolution; 2) running the simulator and hiding its window in background makes it consume less CPU resouces). 

[//]: # (Image References)

[image_1]: ./images/final_version.png "Final Result"

**The final result for my implementation of the Particle Algorithm was position error x = 0.148, y = 0.132; yaw rate error 0.005. The track was completed 49.74s.** (note that the algorithm uses random noise and probability distribution to select best particiles, therefore exact results can very between one execution and another; likewise executiuon time will depend on equipment and CPU load).

![alt text][image_1]

---

**Key files:**
* src/particle_filter.cpp - file containing all Particle Filter algorithm functions
* README.md - (this file) - writeup on project coding, challenges and possible improvements

Additional files:
* clean.sh, build.sh, run.sh - scripts used for cleaning, compilation, running the project
* install-\*.sh - installation scripts for web sockets library for C (used to communicate between Kalman fitler logic and simulator used to visualise/test)
* src/* - source code files provided by udacity that wrap around the Particule Filter algorithm to feed it data from \data dir and communicate with the simulator
* data/* - data including sensor measurements based on which Particle Filter calculates localization of the car

Additional resources:
* Udacity simulator - complimentary to this project, necessary to test it - https://github.com/udacity/self-driving-car-sim/releases

---

# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will need to submit is your `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time).

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
