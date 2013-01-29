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

#include <structureColoring/RoSeConnector/StrColRoSeService.h>

#include <algorithm>
#include <structureColoring/StrColParams.h>

/*****************************************************************************/

RoSe::Service::Ptr StrColRoSeService::create(const RoSe::SID &sid) {
	return new StrColRoSeService(sid);
}

/*****************************************************************************/

StrColRoSeService::StrColRoSeService(const RoSe::SID &sid) :
	RoSe::Service(sid, "StructureColoring_RoSeService"), mNH("~"), mCondition(mMutex) {
}

/*****************************************************************************/

StrColRoSeService::~StrColRoSeService() {
	dispose();
}

/*****************************************************************************/

void StrColRoSeService::terminate() {}

/*****************************************************************************/

void StrColRoSeService::finish() {
	RoSe::Mutex::AutoLocker lock(mMutex);
	mCondition.signal();
	mNH.shutdown();
}

/*****************************************************************************/

void StrColRoSeService::execute() {
    mBBThread.reset(new boost::thread(boost::bind(&StrColRoSeService::blackboardReadRun, this)));
	Planes detectedPlanes;
	Cylinders detectedCylinders;
	while (isRunning()&&ros::ok()) {
		RoSe::Mutex::AutoLocker lock(mMutex);
		mCondition.wait();
		if (mLastPointCloud && isRunning() && ros::ok()) {
			detectedPlanes.clear();
			detectedCylinders.clear();
			//segment pointCloud mLastPointCloud
			PlanePtrs pointMapping(mLastPointCloud->points.size());
			CylinderPtrs cylinderMapping;
			mStrCol.doSegmentation(mLastPointCloud, pointMapping, detectedPlanes, cylinderMapping, detectedCylinders);
			//send unsegmented points
			RosePointcloudPtr unsegmentedPoints(new RosePointcloud);
			extractUnmappedPointsToRosePC(unsegmentedPoints, mLastPointCloud, pointMapping);
			sendPoints(unsegmentedPoints);
			std::cout << unsegmentedPoints->points().size() << " points were not segmented and sent back" << std::endl;
			//send detected planes
			sendPlanes(detectedPlanes);
			mLastPointCloud.reset();
		}
	}
	mBBThread->join();
}

/*****************************************************************************/

void StrColRoSeService::initialize() {
	//uncomment cvError switch for debug
	#ifndef NDEBUG
	cv::setBreakOnError(true);
	#endif

	//get configuration parameters from cfg-file
	const RoSe::ConfigService& cfg = getConfiguration().getService(getID());
	std::cout << "own SID is: " << getID() << std::endl;
	mPointcloudBlackboardSenderSID = cfg.getParamSID("BlackboardSID");
	mSegmentedPlanesAndPointsRecieverSID = cfg.getParamSID("SegmentedPlanesAndPointsRecieverSID");

	StrColParams params;

	//params for all primitivs
	params.mAngleEps = cfg.getParamFloat("AngleEps");
	params.mAngleEpsOnMinOctreeRes = cfg.getParamFloat("AngleEpsOnMinOctreeRes");
	params.mMinPointsInCC = cfg.getParamInteger("MinPointsInCC");
	params.mMinNodesInCC = cfg.getParamInteger("MinNodesInCC");
	params.mTriesOnEachDepth = cfg.getParamInteger("TriesOnEachDepth");
	//histograms
	params.mMaxHTDistanceThreshold = cfg.getParamFloat("MaxHTDistanceThreshold");
	params.mHTOctreeBinSizeFactor = cfg.getParamFloat("HTOctreeBinSizeFactor");
	params.mHTDistanceDeviationFactor = cfg.getParamFloat("HTDistanceDeviationFactor");
	//sphere-histograms
	params.mPhi_resolution = cfg.getParamInteger("PhiResolution");
	//SAC
	params.mMaxSACDistanceThreshold = cfg.getParamFloat("MaxSACDistanceThreshold");
	params.mMinSACDistanceThreshold = cfg.getParamFloat("MinSACDistanceThreshold");
	params.mSACOctreeFactor = cfg.getParamFloat("SACOctreeFactor");
	params.mSACOutlierRatioThreshold = cfg.getParamFloat("SACOutlierRatioThreshold");
	//normals
	params.mMinPointsForNormal = cfg.getParamInteger("MinPointsForNormal");
	params.mCurvThreshold = cfg.getParamFloat("CurvThreshold");
	params.mCurv2Threshold = cfg.getParamFloat("Curv2Threshold");
	params.mPrincipalVarFactor = cfg.getParamFloat("PrincipalVarFactor");
	params.mMinOctreeNodes = cfg.getParamInteger("MinOctreeNodes");
	params.mMinOctreeResolution = cfg.getParamFloat("MinOctreeResolution");
	params.mSqDistFactor = cfg.getParamFloat("SqDistFactor");
	//textures
	params.mTexelSizeFactor = cfg.getParamFloat("TexelSizeFactor");
	params.mDilateIterations = cfg.getParamInteger("DilateIterations");
	//debug and compare
	params.mDebugSteps = cfg.getParamInteger("DebugSteps");
	params.mPclSACmaxIter = cfg.getParamInteger("mPclSACmaxIter");
	params.mOnlyDepth = cfg.getParamInteger("mOnlyDepth");

	//params for planes only
	params.mPostProcessing = cfg.getParamInteger("mPostProcessing");
	params.mMergePlanesSimilarityFactor = cfg.getParamFloat("MergePlanesSimilarityFactor");
	params.mNodeToBBDistance = cfg.getParamFloat("NodeToBBDistance");
	params.mConnectionNeighbors = cfg.getParamInteger("ConnectionNeighbors");
	params.mRho_max = cfg.getParamFloat("RhoMax");
	params.mNodeSegmentedRatio = cfg.getParamFloat("NodeSegmentedRatio");

	//params for cylinders only
	params.mMinNodesInCylinder = cfg.getParamInteger("MinNodesInCylinder");
	params.mNormalDistanceWeight = cfg.getParamFloat("NormalDistanceWeight");
	params.mMinRadiusFactor = cfg.getParamFloat("MinRadiusFactor");
	params.mMaxRadiusFactor = cfg.getParamFloat("MaxRadiusFactor");
	params.mMinCylinderRadius = cfg.getParamFloat("MinCylinderRadius");
	params.mMaxCylinderRadius = cfg.getParamFloat("MaxCylinderRadius");
	params.mCylinderBins = cfg.getParamInteger("CylinderBins");
	params.mCylinderPairNeighbors = cfg.getParamInteger("CylinderPairNeighbors");
	params.mMidPointBinSizeFactor = cfg.getParamFloat("MidPointBinSizeFactor");
	params.mRadiusDevFactor = cfg.getParamFloat("RadiusDevFactor");
	params.mOccupiedRatio = cfg.getParamFloat("OccupiedRatio");
	params.mCylinderHeightDev = cfg.getParamFloat("CylinderHeightDev");

	//params from command line input
	params.mVerbose = cfg.getParamBoolean("Verbose");
	params.mKinect = cfg.getParamBoolean("Kinect");
	params.mLaser = cfg.getParamBoolean("Laser");
	params.mTextures = cfg.getParamBoolean("Textures");
	params.mCylinder = cfg.getParamBoolean("Cylinder");
	params.mPclSAC = cfg.getParamBoolean("PclSAC");
	params.mNoRansacStep = cfg.getParamBoolean("NoRansacStep");
	params.mLoadPCD = cfg.getParamBoolean("LoadPCD");
	params.mPCDnoRGB = cfg.getParamBoolean("LoadPCDnoRGB");
	params.mWriteRawPic = cfg.getParamBoolean("WriteRawPic");
	params.mRawPicFilename = cfg.getParamString("RawPicFilename");
	params.mRawPicCounter = cfg.getParamInteger("RawPicCounter");
	params.mRuntimeFilename = cfg.getParamString("RunTimeFilename");

	mStrCol.setParams(params);
}

/*****************************************************************************/

void StrColRoSeService::receiveMsg(const RoSe::SID&, const RoSe::Message::Ptr) {
}

/*****************************************************************************/

void StrColRoSeService::extractUnmappedPointsToRosePC(RosePointcloudPtr unsegmentedPoints,
		PointCloudPtr pointCloud, PlanePtrs pointMapping) {
	union {
		float rgbfloat;
		uint8_t rgb[4];
	} ColorUnion;
	float origin3f[3] = {0.f, 0.f, 0.f};
	Point3fc origin(origin3f);
	origin.userdata() = RoSe::Color8(0, 0, 0, 0);
	unsegmentedPoints->origin() = origin;
	unsegmentedPoints->points().reserve(pointMapping.size());
	for (unsigned int i = 0; i < pointMapping.size(); ++i) {
		//skip segmented points
		if (pointMapping[i] != 0)
			continue;
		//get coordinates of unsegmented points
		Point3fc p;
		p[0] = pointCloud->points[i].x;
		p[1] = pointCloud->points[i].y;
		p[2] = pointCloud->points[i].z;
		//get color of unsegmented points
		ColorUnion.rgbfloat = pointCloud->points[i].rgb;
		RoSe::Color8 color(ColorUnion.rgb[0], ColorUnion.rgb[1], ColorUnion.rgb[2], ColorUnion.rgb[3]);
		p.userdata() = color;
		//add unsegmented points to unsegPointsPC
		unsegmentedPoints->points().push_back(p);
	}
}

/*****************************************************************************/

void StrColRoSeService::encodeHeightmap(std::vector<unsigned char>& encodedHeightmap, const cv::Mat& heightmap,
		float widthRatio, float heightRatio){
	unsigned int smallWidth = (float) heightmap.cols * (float) widthRatio;
	unsigned int smallHeight = (float) heightmap.rows * (float) heightRatio;
	cv::Mat tempMat(heightmap, cv::Rect(0, 0, smallWidth, smallHeight));
	cv::Mat img;
	tempMat.convertTo(img, CV_8U, 255);
	std::vector<int> params(2);
	params[0] = CV_IMWRITE_PNG_COMPRESSION;
	params[1] = 9;
	cv::imencode(".png", img, encodedHeightmap, params);
}

/*****************************************************************************/

void StrColRoSeService::generateAlphamap(std::vector<bool>& alphamap, const cv::Mat& tempMat){
	alphamap.resize(tempMat.rows * tempMat.cols, false);
	for(int r = 0; r < tempMat.rows; ++r){
		for(int c = 0; c < tempMat.cols; ++c){
			if (tempMat.at<cv::Vec4f>(r, c)[3] > 0.5f)
				alphamap[r * tempMat.cols + c] = true;
		}
	}
}

/*****************************************************************************/

void StrColRoSeService::encodeTexturemap(std::vector<unsigned char>& encodedTexturemap,
		std::vector<bool>& alphamap, const cv::Mat& texturemap, float widthRatio, float heightRatio){
	unsigned int smallWidth = (float) texturemap.cols * (float) widthRatio;
	unsigned int smallHeight = (float) texturemap.rows * (float) heightRatio;
	cv::Mat tempMat(texturemap, cv::Rect(0, 0, smallWidth, smallHeight));
	cv::Mat img;
	tempMat.convertTo(img, CV_8U, 255);
	std::vector<int> params(2);
	params[0] = CV_IMWRITE_PNG_COMPRESSION;
	params[1] = 9;
	cv::imencode(".png", img, encodedTexturemap, params);
	generateAlphamap(alphamap, tempMat);
}

/*****************************************************************************/

void StrColRoSeService::sendPlanes(const Planes& planes) {
	for(Planes::const_iterator pit = planes.begin(); pit != planes.end(); ++pit){
		Point3f normal;
		convertPoint(normal, (*pit)->getPlane3D().getPlaneNormal());
		Points3f brVertices;
		convertPoints(brVertices, (*pit)->getBRVertices());
		std::vector<uchar> encodedHeightmap, encodedTexturemap;
		std::vector<bool> alphamap;
		float widthRatio = (*pit)->getTextureWidthRatio();
		float heightRatio = (*pit)->getTextureHeightRatio();
		encodeHeightmap(encodedHeightmap, (*pit)->getHeightMap(), widthRatio, heightRatio);
		encodeTexturemap(encodedTexturemap, alphamap, (*pit)->getTextureMap(), widthRatio, heightRatio);
		RoSe::TexturedPlaneSegment planeForMsg(normal, (*pit)->getPlane3D().getPlaneDistanceToOrigin(),
				(*pit)->getDistanceThreshold(), brVertices,
				(*pit)->getHeightMap().type(), encodedHeightmap,
				(*pit)->getTextureMap().type(), encodedTexturemap,
				alphamap);
		RoSe::RefPtr<RoSe::MsgTexturedPlaneSegment> Msg(new RoSe::MsgTexturedPlaneSegment(planeForMsg));
		RoSe::Message::Ptr msg(Msg);
		comm().sendTo(mSegmentedPlanesAndPointsRecieverSID, msg);//, RoSe::Message::TOS_ACK, true);
	}
	std::cout << "Textured Planes sent to " << mSegmentedPlanesAndPointsRecieverSID << "." << std::endl;
}

/*****************************************************************************/

void StrColRoSeService::sendPoints(RosePointcloudPtr pointCloud) {
	unsigned int stepsize = 100;//TODO Calculate good stepsize as number of points per Message
	const std::vector<Point3fc> points(pointCloud->points());
	RoSe::Quaternion orientation = pointCloud->orientation();
	Point3fc origin = pointCloud->origin();
	RoSePointCloudPoints::const_iterator start = pointCloud->points().begin();
	RoSePointCloudPoints::const_iterator stop = pointCloud->points().begin();
	unsigned int finished = 0;
		while(finished < points.size()){
		RosePointcloudPtr tmpPC(new RosePointcloud());
		tmpPC->orientation() = orientation;
		tmpPC->origin() = origin;
		if ((int)stepsize > (int)points.size() - (int)finished){
			stepsize = (int)points.size() - (int)finished;
		}
		stop += stepsize;
		finished += stepsize;
		std::copy(start, stop, std::back_inserter(tmpPC->points()));
		start += stepsize;
		RoSe::Message::Ptr msg = new RoSe::MsgPointCloud3fc(*tmpPC);
		comm().sendTo(mSegmentedPlanesAndPointsRecieverSID, msg);//, RoSe::Message::TOS_ACK, true);
	}
}

/*****************************************************************************/

bool StrColRoSeService::connectBlackboard()
{
    try
    {
        mPCBlackboard = RoSe::Blackboard::fromSID(mPointcloudBlackboardSenderSID, false);
        std::cout << "Blackboard connected to: " << mPointcloudBlackboardSenderSID << "." << std::endl;
        return true;
    } catch (RoSe::IPCError &e) {
        ;
    }
    return false;
}

/*****************************************************************************/

void StrColRoSeService::readBlackboard()
{
	union{
		float rgbfloat;
		uint8_t rgba[4];
	}colorUnion;
    if (mPCBlackboard.hasData())
    {
        RoSe::Blackboard::Reader reader = mPCBlackboard.getReader();
        RoSe::Message::Ptr msg = reader.getMessage();
        reader.finish();
        if (RoSe::Message::isType<RoSe::MsgPointCloud3fc>(msg))
        {
            RoSe::MsgPointCloud3fc &msgPC = msg->cast<RoSe::MsgPointCloud3fc>();
            RosePointcloud pointCloud = msgPC.getPayload();
            std::cout << "received " << pointCloud.points().size() << " points.\n";

            RoSe::Mutex::AutoLocker lock(mMutex);
            mLastPointCloud.reset(new PointCloud());
            mLastPointCloud->points.reserve(pointCloud.points().size());
            mLastPointCloud->width = pointCloud.points().size();
            mLastPointCloud->height = 1;
            for (size_t i = 0; i < pointCloud.points().size(); ++i)
            {
                const Point3fc& p = pointCloud.points()[i];
                colorUnion.rgba[0] = p.userdata().red();
                colorUnion.rgba[1] = p.userdata().green();
                colorUnion.rgba[2] = p.userdata().blue();
                colorUnion.rgba[3] = p.userdata().alpha();
                PointT newP;
                newP.x = p[0];
                newP.y = p[1];
                newP.z = p[2];
                newP.rgb = colorUnion.rgbfloat;
                mLastPointCloud->points.push_back(newP);
            }
            std::cout << "\nNew Pointcloud recieved with " << mLastPointCloud->points.size() << " points.\n";
            mCondition.signal();
        }
    }
}

/*****************************************************************************/

void StrColRoSeService::blackboardReadRun()
{
    while (isRunning()&&ros::ok())
    {
        if (mPCBlackboard.isValid())
        {
            try
            {
                mPCBlackboard.wait(100);
                this->readBlackboard();
            }
            catch (RoSe::TimeoutError e)
            { ; }
        }
        else
        {
            if (!this->connectBlackboard()) std::cout << "Could not connect to blackboard with SID: "<< mPointcloudBlackboardSenderSID << ".\n";
            usleep(1000);
        }
    }
    RoSe::Mutex::AutoLocker lock(mMutex);
    mCondition.signal();
}
