# 2D Scan Matching GUI

This is a program written in C# / .NET which performs scan-alignment to align a sequence of scans obtained from a laser range finder.Scan-alignment is a common framework for correct the pose of a mobile robot in an optimisation framework, it is also often used in generating proposal distributions for sequential monte-carlo (SMC) based simultaneous localisation and mapping (SLAM) solutions. 

The scan alignment technique implemented here is similar to [[1]](#1). The same technique is also implemented in https://github.com/nicodell88/Bragg-Edge-Fitting for obtaining the unknown pose-offsets of a poly-crystaline sample within an energy resolved neutron beam.

## Optimisation Routine

Many scan matching techniques require greedy data association. This approach compares all points in the point cloud to one-another. This solution is very similar to the method presented in [[1]](#1). The cost function used is

<img src="https://render.githubusercontent.com/render/math?math=f(\theta)=\sum_{i} \sum_{j} \frac{1}{\sqrt{2 \pi \sigma^{2}}} \exp \left(\frac{-1}{2 \sigma^{2}}\left\|\left[\begin{array}{l}x_{i}(\theta) \\ y_{i}(\theta)\end{array}\right]-\left[\begin{array}{l}x_{j} \\ y_{j}\end{array}\right]\right\|^{2}\right).">

By halving the exponent the resulting cost function can be made to look like a least squares cost function and  quasi newton approximations (see Chapter 8 of [[2]](#2)) can be used to speed up convergence. The optimal robot pose is given by

<img src="https://render.githubusercontent.com/render/math?math=\theta^{\star}=\arg \max _{\theta} f(\theta)">

Where 

<img src="https://render.githubusercontent.com/render/math?math=\theta = \text{is the pose of the robot} (x,y,\psi)">

<img src="https://render.githubusercontent.com/render/math?math=x_i(\theta) y_i(\theta) = \text{are the }  \ (x,y) \ \text{scan points for the current scan as a function of the current pose } \ \theta">

<img src="https://render.githubusercontent.com/render/math?math=x_j,y_j = \text{are the previous} \ (x,y)\ \text{scan points from previous scans (or grid cells).}">

<img src="https://render.githubusercontent.com/render/math?math=\sigma =  \text{a tuning parameter}">

## Data format

The data files contain sensor information, robot pose, and measurements.

* ```LidarAngles,-1.83260,-1.81514, ... ,1.79769,1.81514,1.83260``` - Specifies the direction of each beam (forward-left-up coordinates).

* ``` ODOM x,,y,phi``` - Specifies the initial conditions for the robot pose at that time step.

* ```Lidar,3.43455,3.44317, ... ,3.25120,3.51261,3.46519,3.5600``` - Specifies the range measurements

* ```LidarMaxRange, X``` - Specifies the sensors max range.

  

## Results

The result of appllying the algorithm to 25 scans, with 211 measurements per scan is shown below. The solution does not diverge when it is initialised with the correct pose and the results with a noisy initial pose appear consistent. 

Improvements include:

* Vectorising the inner for loop of the cost function.
* Finding or writing a more appropriate solver. The author has found a trust region method to work well with this cost function and Gauss-newton approximation of the hessian.


![alt text](https://github.com/nicodell88/2D_Scan_Matching_GUI/blob/master/Example.png?raw=true "Logo Title Text 1")



## References

<a id="1">[1]</a>  Biber, P., & Straßer, W. (2003, October). The normal distributions transform: A new approach to laser scan matching. In *Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003)(Cat. No. 03CH37453)* (Vol. 3, pp. 2743-2748). IEEE.

<a id="2">[2]</a> Nocedal, J., & Wright, S. (2006). *Numerical optimization*. Springer Science & Business Media.

