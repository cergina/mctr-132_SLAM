# SLAM in real-time
### Bachelor's degree work
### Keywords: 
ROS, Python, C++, LiDAR, IMU, Jetson TX2, OcTree
### University
Faculty of Informatics and Information Technology, Slovak Technical University, Bratislava
### Academic year
2018/2019

### Permanent link location to work at official central register of final theses of Slovak Republic:
 https://opac.crzp.sk/?fn=detailBiblioForm&sid=F8FE310152E9D1F1EB114741F7FB
Installation guide is there as well.

### Short description: 
This work realizes 3D mapping in real time (X and Y axis tilt) with LiDAR and IMU thats calculated on slow low powered (old smartphone powered) CPU. The resolution of 3D map (level of detail) can be much higher if more powerful CPU is used. It's done via 2D scan, where the point cloud is then translated to IMU's 3D coords and then sent regularly to OcTree which then adds/subtracts points from the generated map. Therefore movement of obstacles is taken into account.

It's possible to extract OBJ map with MTL file and it's also possible to get obstacle detection in 8 directions in 3 height levels, creating 24 3D vectors in which the distance is calculated. 
