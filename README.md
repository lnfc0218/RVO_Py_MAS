# RVO_Py_MAS_3D
3D Reciprocal Velocity Obstacle(RVO) for EL2425 mini drone project, see [our repository](https://github.com/jolilj/el2425_bitcraze)

## Origin
 - Originally forked from [MengGuo/RVO_Py_MAS](https://github.com/MengGuo/RVO_Py_MAS);
 - extended to **three dimensional** scenarios for collision avoidance by waypoint UAV navigation;
 - removed features redundant for our purposes (2D visualization & static obstacles).
 
## TODO

### For Now:

 -[ ]3D simulation & visualization with `matplotlib`;
 -[ ]re-factor loops involving `NumPy` to avoid dependencies!
 -[ ]fine-tune parameters to integrate with [PositionHandler](https://github.com/jolilj/el2425_bitcraze/blob/master/scripts/position_handler.py);
 -[ ]add boundaries;
 -[ ]present RVO algorithm to teammates;
 -[ ]write reports on RVO.
 
### For Future:

 - take kinematics of robot into account;
 - clean up angle comparison，possibly with quaternion;
 - study effects of aperiodic velocity updates;
 - add more obstacle models(fixed, moving, cluttered etc.).
 
## Literature

1. Van Den Berg, Jur, et al. "Reciprocal n-body collision avoidance." Robotics research. Springer Berlin Heidelberg, 2011. 3-19.
2. A., Kamphuis, Implementation of the Velocity Obstacle Principle for 3D Dynamic Obstacle Avoidance in Quadcopter Waypoint Navigation,  2014
 
