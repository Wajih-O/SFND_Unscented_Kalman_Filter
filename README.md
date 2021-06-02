# SFND_Unscented_Kalman_Filter

This project implements an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.




<img src="media/ukf_highway.png" width="500" height="300" />

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway


The `main.cpp` uses `highway.h` to create a straight three lane highway environment with three traffic cars and the main ego car at the center. The viewer scene is centered around the ego car, and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each traffic car has its own UKF object generated for it and will update each one during every time step.

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z-axis is not considered for tracking, so we are only tracking along the X/Y axis.

## NIS output evaluation


The Normalized Innovation Squared is computed and logged into files for both Lidar and Radar sensors and all three cars. Then visualized in [nis output evaluation notebook](notebooks/nis_output_eval.ipynb) for the chosen parameters.

The obtained RMSE values are lower than the tolerance outlined in the project rubric.


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
  * tested PCL version: PCL 1.7
