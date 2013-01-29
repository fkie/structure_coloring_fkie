/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Bastian Gaspers
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer FKIE nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * This file is part of the StructureColoring ROS package.
 *
 * The StructureColoring ROS package is free software:
 * you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The StructureColoring ROS package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with The StructureColoring ROS package.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GUINODE_H_
#define GUINODE_H_

#include <ros/ros.h>
#include <QThread>
#include <structureColoring/gui/StructureGL.h>
#include <structureColoring/StructureColoring.h>

class GuiNode : public StructureColoring, public QThread{
public:
	GuiNode(int argc, char** argv): StructureColoring(argc, argv), mGlWindow(0){
		ros::init(argc, argv, "structureColoring");
		ros::Time::init();
		initNode(*(new ros::NodeHandle));
	}

	virtual ~GuiNode(){}

	void init(StructureGL* glWindow){
		mGlWindow = glWindow;
		start();
	}

	void getUnsegPointCloud (PointCloudPtr unsegPointCloud, const PointCloudPtr pointCloud, const PlanePatchVector& pointMapping){
		assert(pointCloud->points.size() == pointMapping.size());
		unsegPointCloud->points.reserve(pointCloud->points.size());
		for(size_t i = 0; i < pointMapping.size(); i++){
			if (pointMapping[i] != 0)
				continue;
			unsegPointCloud->points.push_back(pointCloud->points[i]);
		}
		unsegPointCloud->width = unsegPointCloud->points.size();
		unsegPointCloud->height = 1;
	}

	void run(){
		ros::Rate loop_rate(10);
		if (waitOnKinectMsgs() || waitOnLaserMsgs())
			while (mNodeHandle->ok()) {
				ros::spinOnce();
				if (mGlWindow && mPointCloud){
					if (mPointCloudMutex.tryLock(100)){
						ROS_INFO("pointcloud(%zu) set.", mPointCloud->points.size());
						mGlWindow->setPointCloud(mPointCloud);
						PointCloudPtr unsegPoints = PointCloudPtr(new PointCloud());
						getUnsegPointCloud(unsegPoints, mPointCloud, mPointMapping);
						mGlWindow->setUnsegmentedPointCloud(unsegPoints);
						ROS_INFO("unsegmented pointcloud(%zu) set.", unsegPoints->points.size());
						mPointCloudMutex.unlock();
					}
					if (mPlanePatchesMutex.tryLock(100)){
						ROS_INFO("plane patches (%zu) set.", mPlanePatches.size());
						mGlWindow->setPlanePatches(mPlanePatches);
						mPlanePatchesMutex.unlock();
					}
					if (mCylinderPatchesMutex.tryLock(100)){
						ROS_INFO("cylinderPatches (%zu) set.", mCylinderPatches.size());
						mGlWindow->setCylinderPatches(mCylinderPatches);
						mCylinderPatchesMutex.unlock();
					}
				}
				loop_rate.sleep();
			}
		else {
			getPointsFromFile();
			if (mGlWindow && mPointCloud){
				if (mPointCloudMutex.tryLock(100)){
					ROS_INFO("pointcloud set.");
					mGlWindow->setPointCloud(mPointCloud);
					mPointCloudMutex.unlock();
				}
				if (mPlanePatchesMutex.tryLock(100)){
					ROS_INFO("plane patches (%zu) set.", mPlanePatches.size());
					mGlWindow->setPlanePatches(mPlanePatches);
					mPlanePatchesMutex.unlock();
				}
			}
			ros::waitForShutdown();
		}
	}

	void stop(){
		ros::shutdown();
	}

private:
	StructureGL* mGlWindow;
};

#endif /* GUINODE_H_ */
