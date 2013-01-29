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

#include <structureColoring/RosVisualization.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

/*****************************************************************************/

RosVisualization::RosVisualization(ros::NodeHandle& nh){
	mHistogramPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/histogramspc2", 5);
	mPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/pc2", 5);
	mSegmentedCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/segmentedcloud2", 5);
	mMarkerPublisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	mCylinderMarkerPublisher = nh.advertise<visualization_msgs::Marker>("cylinder_marker", 100);
	mCylinderPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/cylinderPoints", 5);
	mFrameID = "/openni_rgb_optical_frame";
}

/*****************************************************************************/

void RosVisualization::getColorByIndex(float *rgb, unsigned int index, unsigned int total){
	float t = 1 - (static_cast<float>(index)/static_cast<float>(total));
// set rgb values
	if(t == 1.0){
		rgb[0] = 1.0f;
		rgb[1] = 0.0f;
		rgb[2] = 0.0f;
	} else if (t < 1.0 && t >= 0.83333) {
		rgb[0] = 1.0f;
		rgb[1] = 1.0f-(t-0.83333)/0.16666;
		rgb[2] = 0.0f;
	} else if (t < 0.83333 && t >= 0.66666) {
		rgb[0] = (t-0.66666)/0.16666;
		rgb[1] = 1.0f;
		rgb[2] = 0.0f;
	} else if (t < 0.66666 && t >= 0.5) {
		rgb[0] = 0.0f;
		rgb[1] = 1.0f;
		rgb[2] = 1.0f-(t-0.5)/0.16666;
	} else if (t < 0.5 && t >= 0.33333){
		rgb[0] = 0.0f;
		rgb[1] = 1.0f-(t-0.33333)/0.16666;
		rgb[2] = 1.0f;
	} else if (t < 0.33333 && t >= 0.16666){
		rgb[0] = (t-0.16666)/0.16666;
		rgb[1] = 0.0f;
		rgb[2] = 1.0f;
	} else {
		rgb[0] = 1.0f;
		rgb[1] = 0.0f;
		rgb[2] = 1.0f-t/0.16666;
	}
}

/*****************************************************************************/

void RosVisualization::publishPairMarker(const NodePairs& nodePairs){
	if (mCylinderMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = mFrameID;
	line_list.header.stamp = ros::Time::now();
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.ns = "pairs";
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	line_list.color.r = 1.0;
	line_list.color.g = 0.4;
	line_list.color.b = 0.0;
	line_list.color.a = 1.0;
	const Vec3& centerPoint = nodePairs[nodePairs.size() * 0.7].getFirst()->value_.meanPos;
	for(NodePairs::const_iterator np_it = nodePairs.begin(); np_it != nodePairs.end(); ++np_it){
		if (np_it->getFirst()->value_.meanPos == centerPoint || np_it->getSecond()->value_.meanPos == centerPoint){
			geometry_msgs::Point pn1, pn2;
			Vec3 meanPos = np_it->getFirst()->value_.meanPos;
			pn1.x = meanPos.x();
			pn1.y = meanPos.y();
			pn1.z = meanPos.z();
			meanPos = np_it->getSecond()->value_.meanPos;
			pn2.x = meanPos.x();
			pn2.y = meanPos.y();
			pn2.z = meanPos.z();
			line_list.points.push_back(pn1);
			line_list.points.push_back(pn2);
		}
	}
	mCylinderMarkerPublisher.publish(line_list);
}

/*****************************************************************************/

void RosVisualization::publishPlaneMarker(const PlanePatches& planes, const std::string& namePrefix){
	if (mMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker triangle_list;
	triangle_list.header.frame_id = mFrameID;
	triangle_list.header.stamp = ros::Time::now();
	triangle_list.action = visualization_msgs::Marker::ADD;
	triangle_list.pose.orientation.w = 1.0;
	triangle_list.id = 1;
	triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
	triangle_list.scale.x = 1.0;
	triangle_list.scale.y = 1.0;
	triangle_list.scale.z = 1.0;
	triangle_list.color.a = 0.4;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = mFrameID;
	line_list.header.stamp = ros::Time::now();
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.ns = "plane normals";
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	line_list.color.r = 0.0;
	line_list.color.g = 0.0;
	line_list.color.b = 1.0;
	line_list.color.a = 1.0;
	unsigned int i = 0;
	for (PlanePatches::const_iterator planes_it = planes.begin(); planes_it != planes.end(); planes_it++){
		float rgb[3]={0,0,0};
		getColorByIndex(rgb, i, planes.size());
		PlanePatch::Points brVertices=(*planes_it)->getBRVertices();
		geometry_msgs::Point p0, p1, p2, p3;
		p0.x = brVertices[0].x(); p0.y = brVertices[0].y(); p0.z = brVertices[0].z();
		p1.x = brVertices[1].x(); p1.y = brVertices[1].y(); p1.z = brVertices[1].z();
		p2.x = brVertices[2].x(); p2.y = brVertices[2].y(); p2.z = brVertices[2].z();
		p3.x = brVertices[3].x(); p3.y = brVertices[3].y(); p3.z = brVertices[3].z();
		triangle_list.points.push_back(p0);
		triangle_list.points.push_back(p1);
		triangle_list.points.push_back(p2);
		triangle_list.points.push_back(p2);
		triangle_list.points.push_back(p3);
		triangle_list.points.push_back(p0);
		triangle_list.color.r = rgb[0];
		triangle_list.color.g = rgb[1];
		triangle_list.color.b = rgb[2];
		char ns2[255];
//		sprintf(ns2, " %d (%.2f,%.2f,%.2f)", i, rgb[0], rgb[1], rgb[2]);
		sprintf(ns2, " %d", i);
		std::string name = namePrefix;
		name.append(ns2);
		triangle_list.ns = name.c_str();
		mMarkerPublisher.publish(triangle_list);
		triangle_list.points.clear();
		i++;

		geometry_msgs::Point pn1, pn2;
		pn1.x = (*planes_it)->getPlaneCoG().x();
		pn1.y = (*planes_it)->getPlaneCoG().y();
		pn1.z = (*planes_it)->getPlaneCoG().z();
		const Vec3& planeNormal((*planes_it)->getPlane3D().getPlaneNormal());
		pn2.x = pn1.x + planeNormal.x() * 0.1;
		pn2.y = pn1.y + planeNormal.y() * 0.1;
		pn2.z = pn1.z + planeNormal.z() * 0.1;
		line_list.points.push_back(pn1);
		line_list.points.push_back(pn2);
	}
	mMarkerPublisher.publish(line_list);
}

/*****************************************************************************/

void RosVisualization::publishCylinderMarker(const CylinderPatches& cylinders, const std::string& namePrefix){
	if (mCylinderMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker cylinder;
	cylinder.header.frame_id = mFrameID;
	cylinder.header.stamp = ros::Time::now();
	cylinder.action = visualization_msgs::Marker::ADD;
	cylinder.id = 0;
	cylinder.type = visualization_msgs::Marker::CYLINDER;
	cylinder.color.a = 0.7;
	unsigned int i = 0;
	for (CylinderPatches::const_iterator cit = cylinders.begin(); cit != cylinders.end(); cit++){
		float rgb[3]={0,0,0};
		getColorByIndex(rgb, i, cylinders.size());
		cylinder.color.r = rgb[0];
		cylinder.color.g = rgb[1];
		cylinder.color.b = rgb[2];
		cylinder.scale.x = (*cit)->getRadius() * 2.f;
		cylinder.scale.y = (*cit)->getRadius() * 2.f;
		cylinder.scale.z = (*cit)->getHeight();
//		ROS_ERROR("angle: %f, axis %f %f %f", (*cit)->getRotationAngle(), (*cit)->getRotationAxis().x(), (*cit)->getRotationAxis().y(), (*cit)->getRotationAxis().z() );
		Eigen::Quaternionf q((*cit)->getReTransformRot());
		cylinder.pose.orientation.x = q.x();
		cylinder.pose.orientation.y = q.y();
		cylinder.pose.orientation.z = q.z();
		cylinder.pose.orientation.w = q.w();
		cylinder.pose.position.x = (*cit)->getMidPoint().x();
		cylinder.pose.position.y = (*cit)->getMidPoint().y();
		cylinder.pose.position.z = (*cit)->getMidPoint().z();
//		ROS_INFO("Aufpunkt = (%f, %f, %f)", (*cit)->getPointOnAxis().x(), (*cit)->getPointOnAxis().y(), (*cit)->getPointOnAxis().z());
//		ROS_INFO("Mittelpunkt = (%f, %f, %f)", (*cit)->getMidPoint().x(), (*cit)->getMidPoint().y(), (*cit)->getMidPoint().z());
//		ROS_INFO("TopPoint = (%f, %f, %f)", (*cit)->getTopPoint().x(), (*cit)->getTopPoint().y(), (*cit)->getTopPoint().z());
		char ns2[255];
//		sprintf(ns2, " %d (%.2f,%.2f,%.2f)", i, rgb[0], rgb[1], rgb[2]);
		sprintf(ns2, " %d", i);
		std::string name = namePrefix;
		name.append(ns2);
		cylinder.ns = name.c_str();
		mCylinderMarkerPublisher.publish(cylinder);
		i++;
	}
}

/*****************************************************************************/

void RosVisualization::publishOctreeNormalMarker(const OcTree& octree){
	if (mMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker line_list, cube_list, line_list_red;
	line_list.header.frame_id = cube_list.header.frame_id = line_list_red.header.frame_id = mFrameID;
	line_list.header.stamp = cube_list.header.stamp = line_list_red.header.stamp = ros::Time::now();
	line_list.action = cube_list.action = line_list_red.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = cube_list.pose.orientation.w = line_list_red.pose.orientation.w = 1.0;
	line_list.id = 5;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.005;
	line_list.scale.y = 0.005;
	line_list.color.r = 0; line_list.color.g = 1; line_list.color.b = 0;
	line_list.color.a = 1;
	cube_list.id = 3;
	cube_list.type = visualization_msgs::Marker::CUBE_LIST;
	cube_list.color.r = 0.4;
	cube_list.color.g = 0.4;
	cube_list.color.b = 0.4;
	cube_list.color.a = 0.1;
	line_list_red.id = 7;
	line_list_red.type = visualization_msgs::Marker::CUBE_LIST;
	line_list_red.scale.x = 0.005;
	line_list_red.scale.y = 0.005;
	line_list_red.color.r = 1; line_list_red.color.g = 0; line_list_red.color.b = 0;
	line_list_red.color.a = 1;
	line_list_red.ns = "bad normals";
	for (unsigned int d = 0; d <= octree.getMaxDepth(); d++){
//		if (d == 10){
//			cube_list.color.r = 1.0;
//			cube_list.color.g = 0.0;
//			cube_list.color.b = 0.0;
//		}
		const NodePointerList& nodes = octree.getSamplingMap().find(d)->second;
//markers namespaces
		char ns1[255];
		sprintf(ns1, "cubes on depth %u", d);
		cube_list.ns = ns1;
		char ns3[255];
		sprintf(ns3, "normals on depth %u", d);
		line_list.ns = ns3;

		for (NodePointerList::const_iterator node_it = nodes.begin(); node_it != nodes.end(); ++node_it){
            if ((*node_it)->depth_ != (int)d)
                continue;
			float scale = octree.getCellSizeFromDepth(d); //should be the same for all three coordinate axis
			cube_list.scale.x = scale;
			cube_list.scale.y = scale;
			cube_list.scale.z = scale;

			geometry_msgs::Point p, p1;
			Eigen::Vector4f pos = (*node_it)->pos_key_.getPosition(octree.getOctreePtr());
			p.x = pos(0);
			p.y = pos(1);
			p.z = pos(2);
			cube_list.points.push_back(p);
//center of gravity
			Eigen::Vector3f cog_vec = (*node_it)->value_.meanPos;
			p.x = cog_vec.x();
			p.y = cog_vec.y();
			p.z = cog_vec.z();
//			cube_list.scale.x = .5f;
//			cube_list.scale.y = .5f;
//			cube_list.scale.z = .5f;
//			cube_list.points.push_back(p);
//			std_msgs::ColorRGBA c;
//			c.r = 1.f;//1.f - octreeSamplingMap[d][i]->value.curv;
//			c.g = 0.f;//octreeSamplingMap[d][i]->value.curv;
//			c.b = 0.f;
//			cube_list.colors.push_back(c);
			if ((*node_it)->value_.stable){
				if ((std::isfinite((*node_it)->value_.normal.x()))&&
						(std::isfinite((*node_it)->value_.normal.y()))&&
						(std::isfinite((*node_it)->value_.normal.z()))){
					p1.x = p.x + (*node_it)->value_.normal.x() * scale;
					p1.y = p.y + (*node_it)->value_.normal.y() * scale;
					p1.z = p.z + (*node_it)->value_.normal.z() * scale;
					line_list.points.push_back(p); line_list.points.push_back(p1);
				}
			} else {
				if(::calculateNormal(*node_it, Vec3(0.f, 0.f, 0.f), 20, std::numeric_limits<float>::max(), 0.f, 0.f)){
					if ((std::isfinite((*node_it)->value_.normal.x()))&&
							(std::isfinite((*node_it)->value_.normal.y()))&&
							(std::isfinite((*node_it)->value_.normal.z()))){
						p1.x = p.x + (*node_it)->value_.normal.x() * scale;
						p1.y = p.y + (*node_it)->value_.normal.y() * scale;
						p1.z = p.z + (*node_it)->value_.normal.z() * scale;
						line_list_red.points.push_back(p); line_list_red.points.push_back(p1);
					}
				}
			}
		}
//publish marker
		if ((d <= 10) && (!cube_list.points.empty()))
			mMarkerPublisher.publish(cube_list);
		cube_list.points.clear();
		mMarkerPublisher.publish(line_list);
		line_list.points.clear();
		mMarkerPublisher.publish(line_list_red);
		line_list_red.points.clear();
	}

}

/*****************************************************************************/

void RosVisualization::publishHistogramMarker(const SphereUniformSampling& HTBins, const float& scale, const Eigen::Vector3f& translation){
	if (mMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker arrow;
	arrow.header.frame_id = mFrameID;
	arrow.header.stamp = ros::Time::now();
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.pose.orientation.w = 1.0;
	arrow.id = 128;
	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.scale.x = 0.005;
	arrow.scale.y = arrow.scale.x;
	arrow.color.a = 1;
	std::string ns = "histogram arrows";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	HTBins.getHistogramAsPointCloud(*pc, scale, Eigen::Vector3f(0.f, 0.f, 0.f));
	geometry_msgs::Point p1, p2;
	for(size_t pidx = 0; pidx < pc->points.size(); ++pidx){
		char trans[50];
		sprintf(trans, " trans=(%f, %f, %f)", translation.x(), translation.y(), translation.z());
		char dir[50];
		sprintf(dir, " dir=(%f, %f, %f)", pc->points[pidx].x, pc->points[pidx].y, pc->points[pidx].z);
		arrow.ns = ns.append(trans).append(dir);
		float rgbf = pc->points[pidx].rgb;
		unsigned int rgb = *(unsigned int*)&rgbf;
		ROS_ERROR("rgbuint = %u", rgb);
		arrow.color.r = 2.f * ((rgb & 0xff0000) >> 16)/255.0;
		arrow.color.g = 2.f * ((rgb & 0xff00) >> 8)/255.0;
		arrow.color.b = 2.f * (rgb & 0xff)/255.0;
		ROS_ERROR("r=%u g=%u b=%u", rgb & 0xff, rgb & 0xff00, rgb & 0xff0000);
		ROS_ERROR("r=%f g=%f b=%f", arrow.color.r, arrow.color.g, arrow.color.b);
		p1.x = translation.x();
		p1.y = translation.y();
		p1.z = translation.z();
		p2.x = p1.x + 2.f * pc->points[pidx].x;
		p2.y = p1.y + 2.f * pc->points[pidx].y;
		p2.z = p1.z + 2.f * pc->points[pidx].z;
		arrow.points.push_back(p1);
		arrow.points.push_back(p2);
		mMarkerPublisher.publish(arrow);
		arrow.points.clear();
	}
}

/*****************************************************************************/

void RosVisualization::publishHTBinCloud(const SphereUniformSamplings& HTBins){
	if (mHistogramPC2Publisher.getNumSubscribers() == 0)return;
	pcl::PointCloud<pcl::PointXYZRGB> newCloud;
	for(unsigned int i = 0; i < HTBins.size(); i++){
		HTBins[i].getHistogramAsPointCloud(newCloud, .1f, Eigen::Vector3f(.25f * i, 0, 0));
	}
	sensor_msgs::PointCloud2 newMsg;
	pcl::toROSMsg<pcl::PointXYZRGB>(newCloud, newMsg);
	newMsg.header.frame_id=mFrameID;
	newMsg.header.stamp=ros::Time::now();
	mHistogramPC2Publisher.publish(newMsg);
}

/*****************************************************************************/

void RosVisualization::publishHTBinCloud(pcl::PointCloud<pcl::PointXYZRGB> histogramCloud){
	if (mHistogramPC2Publisher.getNumSubscribers() == 0)return;
	sensor_msgs::PointCloud2 newMsg;
	pcl::toROSMsg<pcl::PointXYZRGB>(histogramCloud, newMsg);
	newMsg.header.frame_id=mFrameID;
	newMsg.header.stamp=ros::Time::now();
	mHistogramPC2Publisher.publish(newMsg);
}

/*****************************************************************************/

/*****************************************************************************/

void RosVisualization::publishCylinderPoints(const CylinderPatches& cylinders, const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > inputPC){
	if (mCylinderPointCloudPublisher.getNumSubscribers() == 0)return;
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

	pcl::PointCloud<pcl::PointXYZRGB> tmpCloud;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(inputPC);
	//fill cylinders into PointCloud
	unsigned int i = 0;
	for(CylinderPatches::const_iterator cit = cylinders.begin(); cit != cylinders.end(); ++cit){
		pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices());
		std::copy((*cit)->getInliers().begin(), (*cit)->getInliers().end(), std::back_inserter(pointIndices->indices));
		extract.setIndices(pointIndices);
		extract.filter(tmpCloud);
		float rgb[3];
		getColorByIndex(rgb, i, cylinders.size());
		unsigned int red = rgb[0] * 255.f + 0.5f;
		unsigned int green = rgb[1] * 255.f + 0.5f;
		unsigned int blue = rgb[2] * 255.f + 0.5f;
		unsigned int color = blue + (green<<8) + (red<<16);
		for(unsigned int pit = 0; pit < tmpCloud.points.size(); ++pit){
			tmpCloud.points[pit].rgb = *(float*)&color;
		}
		pointCloud.header = tmpCloud.header;
		pointCloud += tmpCloud;
		i++;
	}

	sensor_msgs::PointCloud2 segMsg;
	pcl::toROSMsg<PointT>(pointCloud, segMsg);
	segMsg.header.frame_id = mFrameID;
	segMsg.header.stamp=ros::Time::now();
	mCylinderPointCloudPublisher.publish(segMsg);
}

/*****************************************************************************/

void RosVisualization::publishPointCloud(const PointCloud& pointCloud){
	if (mPC2Publisher.getNumSubscribers() == 0)return;
	sensor_msgs::PointCloud2 segMsg;
	pcl::toROSMsg<PointT>(pointCloud, segMsg);
	segMsg.header.frame_id=mFrameID;
	segMsg.header.stamp=ros::Time::now();
	mPC2Publisher.publish(segMsg);
}

/*****************************************************************************/

void RosVisualization::publishPoints(const Points& points, const std::string& nameAppendix, const Vec3& color){
	if (mMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker point_list;
	point_list.header.frame_id = mFrameID;
	point_list.header.stamp = ros::Time::now();
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0f;
	point_list.id = 10;
	point_list.type = visualization_msgs::Marker::POINTS;
	point_list.scale.x = 0.02f;
	point_list.scale.y = 0.02f;
	point_list.scale.z = 0.02f;
	point_list.color.a = 0.7f;
	point_list.color.r = color.x();
	point_list.color.g = color.y();
	point_list.color.b = color.z();
	char ns[255];
	sprintf(ns, "PlanePoints %s", nameAppendix.c_str());
	point_list.ns = ns;
	for (Points::const_iterator pit = points.begin(); pit != points.end(); ++pit){
		geometry_msgs::Point p;
		p.x = pit->x();
		p.y = pit->y();
		p.z = pit->z();
		point_list.points.push_back(p);
	}
	mMarkerPublisher.publish(point_list);
}

/*****************************************************************************/

void RosVisualization::publishCylinderPointsNormals(const PointCloud& partCloud, const pcl::PointCloud<pcl::Normal>& normalCloud, const std::string& name){
	if (mMarkerPublisher.getNumSubscribers() == 0)return;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = mFrameID;
	line_list.header.stamp = ros::Time::now();
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 26;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.0005;
	line_list.scale.y = 0.0005;
	line_list.color.r = 0; line_list.color.g = 1; line_list.color.b = 0;
	line_list.color.a = 1;
	std::string linePrefix = "line ";
	linePrefix.append(name);
	line_list.ns = linePrefix;
	visualization_msgs::Marker point_list;
	point_list.header.frame_id = mFrameID;
	point_list.header.stamp = ros::Time::now();
	point_list.action = visualization_msgs::Marker::ADD;
	point_list.pose.orientation.w = 1.0f;
	point_list.id = 25;
	point_list.type = visualization_msgs::Marker::POINTS;
	point_list.scale.x = 0.002f;
	point_list.scale.y = 0.002f;
	point_list.scale.z = 0.002f;
	point_list.color.a = 0.7f;
	point_list.color.r = 0.f;
	point_list.color.g = 1.f;
	point_list.color.b = 0.5f;
	std::string pointPrefix = "point ";
	pointPrefix.append(name);
	point_list.ns = pointPrefix;
	for (unsigned int i = 0; i < partCloud.points.size(); ++i){
//		if(i % 20 == 0){
		geometry_msgs::Point p, pn;
		p.x = partCloud.points[i].x;
		p.y = partCloud.points[i].y;
		p.z = partCloud.points[i].z;
		point_list.points.push_back(p);
		pn.x = p.x + normalCloud.points[i].normal_x * 0.02f;
		pn.y = p.y + normalCloud.points[i].normal_y * 0.02f;
		pn.z = p.z + normalCloud.points[i].normal_z * 0.02f;
		line_list.points.push_back(p); line_list.points.push_back(pn);
//		}
	}
	mMarkerPublisher.publish(point_list);
	mMarkerPublisher.publish(line_list);
}

/*****************************************************************************/

void RosVisualization::printTopics()
{
    ROS_INFO("Fixed Frame: %s\n\tHistogramPC2: %s\n\tInputPC2: %s\n\tSegmentedPlanesPC2: %s\n\tSegmentedCylindersPC2: %s\n\tMarkers: %s\n\tCylinderMarkers: %s", mFrameID.c_str(), mHistogramPC2Publisher.getTopic().c_str(), mPC2Publisher.getTopic().c_str(), mSegmentedCloudPublisher.getTopic().c_str(), mCylinderPointCloudPublisher.getTopic().c_str(), mMarkerPublisher.getTopic().c_str(), mCylinderMarkerPublisher.getTopic().c_str());
}

