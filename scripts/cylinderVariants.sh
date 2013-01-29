#!/bin/bash
./kinectCylinders.sh kinectCylinder.params
read -p "Finished ours. Press enter to start noRansac"
./kinectCylinders.sh kinectCylinderNoRansac.params --disableRansacStep
read -p "Finished noRansac. Press enter to start Octree8"
./kinectCylinders.sh kinectCylinderOctree8.params --depth 8
read -p "Finished Octree8. Press enter to start Octree9"
./kinectCylinders.sh kinectCylinderOctree9.params --depth 9
read -p "Finished Octree9. Press enter to start Octree10"
./kinectCylinders.sh kinectCylinderOctree10.params --depth 10 
#read -p "Finished Octree10. Press enter to start RansacOnly"
#./kinectCylinders.sh kinectCylinderRansacOnly.params --pclSAC 10000
