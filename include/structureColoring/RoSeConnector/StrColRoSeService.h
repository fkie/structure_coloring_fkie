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

#ifndef STRCOLROSESERVICE_H_
#define STRCOLROSESERVICE_H_

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <RoSe/Blackboard.h>
#include <RoSe/Color8.h>
#include <RoSe/Condition.h>
#include <RoSe/Config.h>
#include <RoSe/ConfigService.h>
#include <RoSe/Message.h>
#include <RoSe/MsgPointCloud3fc.h>
#include <RoSe/Mutex.h>
#include <RoSe/PointCloud.h>
#include <RoSe/Quaternion.h>
#include <RoSe/Service.h>
#include <RoSe/SID.h>
#include <RoSe/MsgTexturedPlaneSegment.h>
#include <structureColoring/StructureColoring.h>

class StrColRoSeService : public RoSe::Service {
private:
//typedefs
	typedef Eigen::Vector3f Vec3;
	typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > EigenPoints;

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef boost::shared_ptr<PointCloud> PointCloudPtr;
	typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    typedef RoSe::PointT<float, 3, RoSe::Color8> Point3fc;
	typedef RoSe::PointCloud<Point3fc, RoSe::Quaternion> RosePointcloud;
	typedef RoSe::PointCloud<Point3fc, RoSe::Quaternion>::Points RoSePointCloudPoints;
	typedef boost::shared_ptr<RosePointcloud> RosePointcloudPtr;

	typedef RoSe::PointT<float, 3> Point3f;
	typedef std::vector<Point3f> Points3f;

	typedef StructureColoring::PlanePatchVector PlanePtrs;
	typedef StructureColoring::PlanePatches Planes;
	typedef StructureColoring::CylinderPatchVector CylinderPtrs;
	typedef StructureColoring::CylinderPatches Cylinders;

	typedef boost::scoped_ptr<boost::thread> MyThreadPtr;

//ros communication
	ros::NodeHandle mNH;
//RoSe communication
	RoSe::SID mPointcloudBlackboardSenderSID;
	RoSe::SID mSegmentedPlanesAndPointsRecieverSID;
//thread and inter-thread communication
    RoSe::Mutex mMutex;
    RoSe::Condition mCondition;
	RoSe::Blackboard mPCBlackboard;
    MyThreadPtr mBBThread;
//data
    PointCloudPtr mLastPointCloud;

//computing object
    StructureColoring mStrCol;
protected:
	void initialize();
	void execute();
	void terminate();
	void stop();

	void receiveMsg(const RoSe::SID&, const RoSe::Message::Ptr);

    static void convertPoint(Point3f& outPointRoSe, const Vec3& inPointEigen);
    static void convertPoints(Points3f& brVertices, const EigenPoints& points);
    static void extractUnmappedPointsToRosePC(RosePointcloudPtr unsegmentedPoints, PointCloudPtr mLastPointCloud, PlanePtrs pointMapping);
    static void generateAlphamap(std::vector<bool>& alphamap, const cv::Mat& tempMat);
    static void encodeHeightmap(std::vector<unsigned char>& encodedHeightmap, const cv::Mat& heightmap,
    		float widthRatio, float heightRatio);
    static void encodeTexturemap(std::vector<unsigned char>& encodedTexturemap, std::vector<bool>& alphamap,
    		const cv::Mat& texturemap, float widthRatio, float heightRatio);
    void sendPlanes(const Planes& planes);
    void sendPoints(RosePointcloudPtr pointCloud);

    void blackboardReadRun();
	bool connectBlackboard();
	void readBlackboard();

public:

	/**
	 * Creates a new rosconnectorServer service. A service class must have a
	 * constructor with the same parameter list. It is needed for the
	 * Service.cmdLine() method.
	 *
	 * @param sid    the sid of this service
	 */
	StrColRoSeService(const RoSe::SID &sid);

	virtual ~StrColRoSeService();
	virtual void finish();

	/**
	 * Creates a new service of the specified type;
	 */
	static Service::Ptr create(const RoSe::SID &sid);
};

/*****************************************************************************/

inline void StrColRoSeService::convertPoint(Point3f& outPointRoSe, const Vec3& inPointEigen){
	outPointRoSe[0] = inPointEigen.x();
	outPointRoSe[1] = inPointEigen.y();
	outPointRoSe[2] = inPointEigen.z();
}

/*****************************************************************************/

inline void StrColRoSeService::convertPoints(Points3f& brVertices, const EigenPoints& points){
	brVertices.reserve(4);
	for(EigenPoints::const_iterator pit = points.begin(); pit != points.end(); ++pit){
		Point3f p;
		convertPoint(p, *pit);
		brVertices.push_back(p);
	}
}

/*****************************************************************************/

#endif /* STRCOLROSESERVICE_H_ */
