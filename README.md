# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---


In this project we are going to use Extended Kalman Filter for tracking objects around self driving car. Laser and Radar data will be used as input to extended Kalman Filter. 


Pipeline
---



*The overall pipeline along with the results will be described here!*

<br>

I. At first sensor data will be read and Extended Kalman Filter will be initialized by the first data point. Depending on the sensor type, we apply conversion to convert polar coordinate data into cartesian. For your note, Radar data is in polar coordinates and Laser data is in cartesian coordinates.


* In order to avoid division by zero, I make sure that the data points are not smaller than a threshold value (0.0001). Also in order to avoid recomputing some of the values, they are precomputed and later on used whenever need be.

II. After initializing kalman filter class, object location(Px, Py) and velocity(Vx, Vy) will be predicted.

III. Once the location is predicted, sensor data will be used to update the location of the object. Based on the sensor type (Laser, Radar), there are two different types of update step. If the sensor type is laser, the update step is like typical kalman filter update step. However, if the sensor type is Radar, we have to mitigate the nonlinearity associated with the Radar measurments. This will be done by using jacobian matrix.

* During update step for Radar, we also make sure that the updated φ is in a certain range [-π, π]. 


* After each update step, updated values will be pushed into an estimate array in order to calculate the root mean square error(rmse) between the predicted values and actual ground truth values. As can be seen in the following gif video, the errors for (Px, Py, Vx, Vy) are [0.0954, 0.0837, 0.4499, 0.4357] at the end of the simulation.

<p align="center">
<img src="https://j.gifs.com/gLyr3l.gif" width = "600" />
</p>

IV. I also performed simulations for cases were just one of the sensors was active and measurements of the other senor was not considered during update and prediction step. We observe higher rmse when one of the sensors is inactive, this is because we have less information about the object, as a result there will be more uncertainty in update step and the performance will be degraded. Comparing Laser and Radar based on rmse, it appears that laser performs better because it has lower rmse error. Here is table comparing rmse values for sensor fusion, laser, and radar.

<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/EAdp8r0g58M/0.jpg)](https://www.youtube.com/watch?v=EAdp8r0g58M) -->
<!-- [![Demo Sensor Fusion](https://j.gifs.com/ZVZwnv.gif)](https://www.youtube.com/watch?v=EAdp8r0g58M) -->



<!-- <p align="center"><img src="image/table.png" width = "600" alt="Combined Image" /> -->
</p>

</br>
<br></br>


