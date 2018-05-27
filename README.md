# CarND-P7-Unscented-Kalman-Filter
Udacity Self-Driving Car Nanodegree - Unscented Kalman Filter Implementation

## Overview
This project consists of implementing an [Unscented Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter) with C++. A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Unscented Kalman Filter[UKF] must fusion those measurements to predict the position of the object. The communication between the simulator and the UKF is done using the [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the UKF side. To get this project started, Udacity provides a seed project that could be found [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).

## Prerequisites
The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

This particular implementation is done on Linux OS. In order to install the necessary libraries, use the install-ubuntu.sh.

## Compiling and executing the project
These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `make`: This will create an executable `UnscentedKF`

## Running the Filter
From the build directory, execute `./UnscentedKF`. The output should be:
```
Listening to port 4567
Connected!!!
```
As you can see, the simulator connect to it right away.

The following is a snapshot of the simulator after running my final implementation using both liar and radar measurements
![](https://github.com/JuAnne/CarND-P7-Unscented-Kalman-Filter/blob/master/output/lidar_and_radar.PNG)

Based on the provided data, my UKF will generate the below RMSE outcome
```
px, py, vx, vy = [0.0682, 0.0809, 0.1719, 0.2027]
```
which meets the Accuracy CRITERIA: 
```
px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] 
```

I also ran the simulator based on only lidar or radar data at a time, and here are the results:

Lidar only - 

![](https://github.com/JuAnne/CarND-P7-Unscented-Kalman-Filter/blob/master/output/lidar_only.PNG)

Radar only - 

![](https://github.com/JuAnne/CarND-P7-Unscented-Kalman-Filter/blob/master/output/radar_only.PNG)

As well as my previous EKF for comparisons - 

![](https://github.com/JuAnne/CarND-P6-Extended-Kalman-Filter/blob/master/images/rmse.PNG)

As expected, the UKF that uses both lidar and radar sensor measurements is more accurate than using one sensor at a time, and also achieves lower RMSE result than EKF.

## Visualization
Below are the plots for Lidar and Radar NIS (Normalized Innovation Squared), which shows the UKF is consistent based on the provided data set.

![](https://github.com/JuAnne/CarND-P7-Unscented-Kalman-Filter/blob/master/output/NIS_Lidar.PNG)
![](https://github.com/JuAnne/CarND-P7-Unscented-Kalman-Filter/blob/master/output/NIS_radar.PNG)
