# 2D Scan Matching GUI

This is a program written in C# / .NET which performs scan-alignment to align a sequence of scans obtained from a laser range finder.Scan-alignment is a common framework for correct the pose of a mobile robot in an optimisation framework, it is also often used in generating proposal distributions for sequential monte-carlo (SMC) based simultaneous localisation and mapping (SLAM) solutions. 

The scan alignment technique implemented here is similar to [[1]](#1). The same technique is also implemented in https://github.com/nicodell88/Bragg-Edge-Fitting for obtaining the unknown pose-offsets of a poly-crystaline sample within an energy resolved neutron beam.

TODO: cost function

TODO: how GUI works

TODO: data format


![alt text](https://github.com/adam-p/markdown-here/raw/master/src/common/images/icon48.png "Logo Title Text 1")



## References

<a id="1">[1]</a>  Biber, P., & Straßer, W. (2003, October). The normal distributions transform: A new approach to laser scan matching. In *Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003)(Cat. No. 03CH37453)* (Vol. 3, pp. 2743-2748). IEEE.

<a id="2">[2]</a> Nocedal, J., & Wright, S. (2006). *Numerical optimization*. Springer Science & Business Media.

