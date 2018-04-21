# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

## Objective

The goal of this project is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The Root Mean Squared Error (RMSE) is calculated for every estimation.

A successful implementation requires:

* Compilation on any platform (Linux, Mac, Windows)
* Follow the Prediction -> Update cycle using the motion and observation equations of a 2d Kalman Filter.
* Handling the first measurement appropriately to account for the fact that there is no prior prediction.
* Handling radar measurements via the Extended Kalman Filter approximation.
* Obtaining RMSE values that are lower than [.11, .11, 0.52, 0.52] for the x, y positions and x and y velocities respectively.
* Performing all of the above in an efficient manner.

[image1]: ./RadarH.png
[image2]: ./SimulationRMSE.gif

### Here I will consider each point individually and explain how I addressed them in my implementation

#### 1. Compilation on any platform

The starter code provided for the project was already meant to compile correctly in Linux, Mac and Windows; although compilation in Windows required installing Windows 10 Bash on Ubuntu or using Docker to create a virtual environment.

Since my current platform is Windows but I have low resources, I decided to go for a native approach to avoid the installation of such resource-intensive environments. This decision had a non-trivial time cost, which I intend to contribute to the Nanodegree Program for others in my position. There is little in the repository that will give my choice away, exept for a couple of lines at the end of the CMakeLists.txt file where the names of the libraries are redefined when a Windows environment is found.

The main program file (main.cpp) was modified slightly to accomodate api changes in uWS 0.14.4. This file receives a measurement package message from the simulator and passes it to the FusionEKF class, where the information is processed.

#### 2. Follow the process of a 2 dimensional Kalman Filter

The process is contained in the FusionEKF::ProcessMeasurement function implemented in the FusionEKF.cpp file. The handling of the initial measurement is done in lines 71-97, followed by a prediction step (lines 108-121) and an update step (127-138). The equations of the Kalman Filter are implemented in the KalmanFilter class' (kalman_filter.{h,cpp}) Update and Predict methods, although some of the necessary values are set from the calling ProcessMeasurement method as well as the initialization of the FusionEKF class.

The initialization of the FusionEKF class for example, sets the Radar and Lidar covariance matrices (which in theory are provided by the device manufacturers) and some sensible default values in prepraration of the new measurements. In particular, the state covariance positions (2,2) and (3,3) for the speed covariances were set to 1000 to help it converge faster according to observations after several runs.

#### 3. Handles first measurements appropriately

The logic in FusionEKF::ProcessMeasurement includes a flag to be able to tell when the state (x) variable has been initialized. The first measurement is never used to predict since there is no previous state to compare against.

This assertion also applies to the timestamp used to calculate the time delta.

So the code first calculates delta t and stores the last measurement's timestamp. Afterwards, it initializes the state vector with the readings from the sensor. Care has to be taken to differentiate the sensor type in order to convert from polar to cartesian coordinates in the case of radar measurements.

#### 4. Handles radar measurements via the Extended Kalman Filter approximation

From the FusionEKF class, little can be seen about the Exended Kalman Filter except for the fact that if the sensor type is radar, an additional function call is made to calculate a Jacobian and the function to update is `UpdateEKF` instead of the normal `Update` for laser.

A Jacobian is a method to linearly approximate a function using it's first order derivative. It is a simplification of a Taylor Series Expansion and it is required because:

1. We use a non-linear function to map the state's belief into the measurement space
2. Using a non-linear function to map our Gaussian probability distribution would yield a non-Gaussian distribution that would break the Kalman Filter.

The Jacobian is calculated in the Tools (tools.cpp) class. It requires passing the current (predicted) state and will calculate the Jacobian for the state mapping function h(x):

![state projection function from cartesian to polar coordinates][image1]

The differences between regular Kalman Filter and Extended Kalman Filter become more evident when comparing the update methods from the KalmanFilter class. In the UpdateEKF method, the first step is to map the current state's belief (in cartesian coordinates) to the measurement space (in polar coordinates). The Kalman Filter equations can then be applied to get the updated predicion.

A critical step in the extended Kalman Filter (or any involving polar coordinates really) is normalizing the angles. In the code this is achieved efficiently by using a combination of trigonometrical functions that can be seen in lines 47 and 55 of kalman_filter.cpp.

#### 5. Obtain low RMSE values

Using the simulator, RMSE values obtained after a complete run were below the maximums allowed ([.11, .11, 0.52, 0.52]).

![final frame of the simulator run displaying the RMSE values][image2]

#### 6. Efficient code

As much as possible, the code performs each operation once. For example, in FusionEKF.cpp:113-119 where the time delta is used power 4 and divided by 4 in several operations so the individual expression is calculated once and the result used later.

Other examples are the Update functions in kalman_filter.cpp that perform transpose and inverse operations only once and reuse the result as needed.

## This repository

This project uses the following:

* CMake 3.5 and Make 4.1 (3.81 for Windows)
* Zlib, OpenSSL, and Libuv.
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
* The Udacity Self-Driving Car Engineer Nanodegree Program Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
