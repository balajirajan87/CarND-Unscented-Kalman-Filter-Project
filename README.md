# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project an Unscented Kalman Filter was used to track various objects given noisy lidar and radar measurement data. The UKF was able to succesfully fuse the two different kinds of measurements together even though lidar was in cartesian coordinates and radar was in polar. Although radar was not as accurate spatially as lidar, it could directly measure the velocity of objects. 

## UKF Performance on data
#### sample-laser-radar-measurement-data-1.txt 

Parameters

std_a_ = 3;

std_yawdd_ = 4;

Accuracy - RMSE:

0.0745208

0.0897507

0.390431

0.340212

#### sample-laser-radar-measurement-data-2.txt

Parameters

std_a_ = 3;

std_yawdd_ = 4;

Accuracy - RMSE:

0.0879

0.0788

0.6872

0.3789


### Visuals

All provided data samples can be visually inspected by using the included ukf-visualization jupyter notebook file which also includes graphs for LASER and RADAR NIS values.

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/4d0420af-0527-4c9f-a5cd-56ee0fe4f09e)
for instructions and the project rubric.
