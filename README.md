# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program
* The purpose of this project is using kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

[//]: # (Image References)
[image1]: ./refer_images/EKF_system.png 
[image2]: ./refer_images/TwoStep_Estimation.png 
[image3]: ./refer_images/StatePrediction.png
[image4]: ./refer_images/SensorFusion.png
[image5]: ./refer_images/Laser1.png
[image6]: ./refer_images/Radar1.png
[image7]: ./refer_images/Radar2.png
[image8]: ./refer_images/RMSE.png


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Sensor Fusion Basic Concept

* Sensor Comparison 

	|               |       Camera      |        LIDAR      |        RADAR      | 
	|:-------------:|:-----------------:|:-----------------:|:-----------------:|
	|  Resolution   |        Good       |         OK        |         BAD       | 
	|     NOISE     |        Good       |         BAD       |         BAD       | 
	|   Velocity	|         BAD       |         BAD       |        Good       | 
	|  All-Weather  |         BAD       |         BAD       |        Good       | 
	|     Size      |        Good       |         BAD       |        Good       | 
	
	* A Vehicle or a Robot has sensor to detect the environment all around itself. 
	* Sensor Fusion is using the sensor to obtain the data all around itself and a way to combine the information to obtain the coherent picture. 
	* Each Sensor has strength and weekness, so using Sensor fuction, we can obtain strength of all the sensor to control the vehicle. 

## Overall System of Extended Kalman Filter
	
![alt text][image1]

* Simplify the diagram 

![alt text][image2]

* two step estimation

	* State Prediction - Motion Update
		* Theorem of Total Probability (Addition Theorem)
		
	* Measurement Update
		* Bayes Rule		

### 1. State Prediction
![alt text][image3]

### 2. Measurement Update - LIDAR or RADAR
![alt text][image4]

* LIDAR
	* Measuring the pedestrain position with high accuracy. 
	* Since the measuring data linear, Kalman Filter is sufficent enough.
![alt text][image5]

* RADAR
	* Measuring Velocity using doppler effect
	* Extended Kalman Filter is used to limearized the h(x) fuction with Jacobian Matrix.
![alt text][image6]
![alt text][image7]

## Measuring Performance
* RMSE (Root Mean Squared Error ) measure the deviation of the estimated state from the true state
![alt text][image8]