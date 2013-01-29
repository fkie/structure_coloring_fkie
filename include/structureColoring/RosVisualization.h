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

#ifndef ROSVISUALIZATION_H_
#define ROSVISUALIZATION_H_

#include <structureColoring/OcTree.h>
#include <structureColoring/NodePair.h>
#include <structureColoring/histograms/SphereUniformSampling.h>
#include <structureColoring/structures/PlanePatch.h>
#include <structureColoring/structures/CylinderPatch.h>
#include <structureColoring/NormalEstimationValueClass.h>
#include <octreelib/spatialaggregate/octree.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

//TODO make topics and frame configurable
class RosVisualization{
public:
	typedef PlanePatch::PlanePatches PlanePatches;
	typedef std::vector<PlanePatch::PlanePatchPtr> PlanePatchVector;
	typedef CylinderPatch::CylinderPatches CylinderPatches;
	typedef algorithm::OcTreeSamplingMap<float, NormalEstimationValueClass> OctreeSamplingMap;
	typedef std::vector<SphereUniformSampling> SphereUniformSamplings;
	typedef PlanePatch::PointIndices PointIndices;
	typedef Eigen::Vector3f Vec3;
	typedef PlanePatch::Points Points;
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef OcTree::NodePointers NodePointers;
	typedef OcTree::NodePointerList NodePointerList;

	RosVisualization(ros::NodeHandle& nh);

	static void getColorByIndex(float *rgb, unsigned int index, unsigned int total);

	void setFrame(const std::string& frame_id){mFrameID = frame_id;}

	void publishPairMarker(const NodePairs& nodePairs);
	void publishPlaneMarker(const PlanePatches& planes, const std::string& namePrefix = "plane ");
	void publishCylinderMarker(const CylinderPatches& cylinders, const std::string& namePrefix = "cylinder ");
	void publishOctreeNormalMarker(const OcTree& octree);
	void publishHistogramMarker(const SphereUniformSampling& HTBins, const float& scale, const Eigen::Vector3f& translation);
	void publishHTBinCloud(const SphereUniformSamplings& HTBins);
	void publishHTBinCloud(PointCloud histogramCloud);
    template<typename PointAssignment, typename StructureContainer>
	void publishSegmentedPointcloud(const PointAssignment& segmented, const StructureContainer& planes, const PointCloud& pointCloud);
	void publishCylinderPoints(const CylinderPatches& cylinders, const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > inputPC);
	void publishPointCloud(const PointCloud& pointCloud);
	void publishPoints(const Points& points, const std::string& nameAppendix, const Vec3& color);
	void publishCylinderPointsNormals(const PointCloud& partCloud, const pcl::PointCloud<pcl::Normal>& normalCloud, const std::string& name = "no identifier");
    void printTopics();

private:
	ros::Publisher mHistogramPC2Publisher;
	ros::Publisher mPC2Publisher;
	ros::Publisher mSegmentedCloudPublisher;
	ros::Publisher mCylinderPointCloudPublisher;
	ros::Publisher mMarkerPublisher;
	ros::Publisher mCylinderMarkerPublisher;
	std::string mFrameID;
	std::string mLaserTopic;
};

template<typename PointAssignment, typename StructureContainer>
void RosVisualization::publishSegmentedPointcloud(const PointAssignment& segmented, const StructureContainer& planes, const PointCloud& pointCloud)
{
	if (mSegmentedCloudPublisher.getNumSubscribers() == 0)return;

	pcl::PointCloud<pcl::PointXYZRGB> segCloud;
	for (unsigned int i=0; i<segmented.size(); i++){

//		segCloud.points.push_back( pointCloud.points[i] );

		if (!segmented[i]){
			unsigned int rgb = 0x000000;
			pcl::PointXYZRGB p;
			p.x = pointCloud.points[i].x;
			p.y = pointCloud.points[i].y;
			p.z = pointCloud.points[i].z;
			p.rgb = *(float*)&rgb;
			segCloud.points.push_back(p);
		}
	}

	unsigned int i=0;
	for (typename StructureContainer::const_iterator pit = planes.begin(); pit != planes.end() ; pit++){
		float rgb[3]={0.f,0.2f,0.5f};
		getColorByIndex(rgb, i, planes.size());
		unsigned int red = rgb[0] * 255.f + 0.5f;
		unsigned int green = rgb[1] * 255.f + 0.5f;
		unsigned int blue = rgb[2] * 255.f + 0.5f;
		unsigned int color = blue + (green<<8) + (red<<16);
		PointIndices inlierIndices = (*pit)->getInliers();
		for (PointIndices::iterator it=inlierIndices.begin(); it != inlierIndices.end(); it++){
			pcl::PointXYZRGB p;
			p.x = pointCloud.points[*it].x;
			p.y = pointCloud.points[*it].y;
			p.z = pointCloud.points[*it].z;
			p.rgb = *(float*)&color;
			segCloud.points.push_back(p);

		}
		i++;
	}

	sensor_msgs::PointCloud2 segMsg;
	pcl::toROSMsg<pcl::PointXYZRGB>(segCloud, segMsg);
	segMsg.header.frame_id=mFrameID;
	segMsg.header.stamp=ros::Time::now();
	mSegmentedCloudPublisher.publish(segMsg);
}

#endif /* ROSVISUALIZATION_H_ */
