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

#include <structureColoring/StructureColoring.h>
#include <structureColoring/grids/CylinderGridMap.h>
#include <structureColoring/RosVisualization.h>
#include <structureColoring/NodePair.h>
#include <structureColoring/histograms/SphereUniformSampling.h>
#include <structureColoring/histograms/TriangleDistributionHistogram.h>
#include <structureColoring/histograms/TriangleDistributionHistogramWithRemove.h>
#include <structureColoring/histograms/TriangleDistributionHistogram2D.h>
#include <structureColoring/histograms/WeightedIdxVector.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <Eigen/StdVector>


/*****************************************************************************/

StructureColoring::StructureColoring(ros::NodeHandle& n, int argc, char* argv[]) : mVis(NULL)
{
    init(n, argc, argv);
}

/*****************************************************************************/

void StructureColoring::init(ros::NodeHandle& n, int argc, char* argv[])
{
	init(argc, argv);
	initNode(n);
	mVis = new RosVisualization(n);
    if (mParams.mVerbose)
    {
        ROS_INFO("Verbose output enabled");
        mVis->printTopics();
    }
}

/*****************************************************************************/

void StructureColoring::init(int argc, char* argv[])
{
	initParams();
	std::string paramFilename;
	parseInput(paramFilename, mParams.mInputFilename, mParams.mOutputFilename, argc, argv);
	if (mParams.mInputFilename == "")
		mParams.mInputFilename = "../ABW/abw.test.0";
	if (mParams.mABW)
		mParams.mInputFilename.append(".range");
	if (mParams.mOutputFilename == "")
		mParams.mOutputFilename = "output";
	mParams.mNormalOutputFilename = mParams.mOutputFilename;
	mParams.mOutputFilename.append(".ms-seg");
	mParams.mNormalOutputFilename.append(".ms-nor");
	mParams.parseParamFile(paramFilename);
	initOctree();
}

/*****************************************************************************/

void StructureColoring::init()
{
	initParams();
	initOctree();
}

/*****************************************************************************/

StructureColoring::~StructureColoring()
{
	if(mVis) delete mVis;
}

/*****************************************************************************/

void StructureColoring::parseInput(std::string& paramFilename, std::string& inputFilename, std::string& outputFilename,
		int argc, char* argv[])
{
	for (int i = 1; i < argc; i += 2) {
		if (std::string(argv[i]) == "--kinect") {
			mParams.mKinect = true;
			mParams.mTextures = true;
			i--;
			continue;
		}
        else if (std::string(argv[i]) == "--textures") {
        	mParams.mTextures = true;
			i--;
			continue;
		}
        else if (std::string(argv[i]) == "--cylinder") {
        	mParams.mCylinder = true;
			i--;
			continue;
		}
        else if (std::string(argv[i]) == "--disableRansacStep") {
        	mParams.mNoRansacStep = true;
            i--;
            continue;
        }
        else if (std::string(argv[i]) == "-v") {
        	mParams.mVerbose = true;
			i--;
			continue;
		}
		if (i + 1 < argc) {
			if (std::string(argv[i]) == "--kinectt") {
				mKinectTopic = argv[i + 1];
				mParams.mKinect = true;
				mParams.mTextures = true;
				continue;
			} else if (std::string(argv[i]) == "--postprocessing") {
				mParams.mPostProcessing = atoi(argv[i + 1]);
			} else if (std::string(argv[i]) == "--debugsteps") {
				mParams.mDebugSteps = atoi(argv[i + 1]);
			} else if (std::string(argv[i]) == "-r") {
				inputFilename = argv[i + 1];
				mParams.mPGM = true;
			} else if (std::string(argv[i]) == "--laser") {
				mParams.mLaser = true;
				mParams.mLaserTopic = argv[i+1];
			} else if (std::string(argv[i]) == "-p") {
				paramFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "-m") {
				outputFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--prefix") {
				mParams.mABW = true;
				inputFilename = outputFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--percPrefix") {
				mParams.mPerceptron = true;
				inputFilename = outputFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--rawpic") {
				mParams.mWriteRawPic = true;
				mParams.mRawPicFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--pcd") {
				mParams.mTextures = true;
				mParams.mLoadPCD = true;
				inputFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--pcdNoRgb") {
				mParams.mLoadPCD = true;
				mParams.mPCDnoRGB = true;
				inputFilename = argv[i + 1];
            } else if (std::string(argv[i]) == "--pclSAC") {
            	mParams.mPclSAC = true;
                mParams.mPclSACmaxIter = atoi(argv[i+1]);
			} else if (std::string(argv[i]) == "--runtimeFile") {
				mParams.mRuntimeFilename = argv[i + 1];
			} else if (std::string(argv[i]) == "--depth") {
				mParams.mOnlyDepth = atoi(argv[i + 1]);
			}
            else {
				std::cout << "Not enough or invalid arguments, please try again.\n"
						<< "arguments are:\n"
						<< "--kinect               Start segmentation on kinect topics\n"
						<< "--kinectt [string]     Start segmentation on kinect, specifying topic\n"
						<< "--laser [string]       Start segmentation on laser [string] topic\n"
						<< "--textures             Start segmentation and calculate textures for planes afterwards,\n"
						<< "                            works on PointClouds with RGB Data ONLY!\n"
						<< "--cylinder             Start segmentation, do cylinder segmentation after plane segmentation\n"
						<< "--disableRansacStep    Start segmentation, but leave out ransac steps#\n"
						<< "-v                     Start segmentation with verbose output (of ros frames)\n"
						<< "--postprocessing [int] Start segmentation with postprocessing steps according to [int]\n"
						<< "--debugsteps [int]     Start segmentation with [int] steps for debugging\n"
						<< "-r [string]            set input PGM filename to [string]\n"
						<< "-p [string]            set param file to [string], 3 params can be configured there:\n"
						<< "                           	mMaxHTDistanceThreshold(float), mHTOctreeBinSizeFactor(float)\n"
						<< "                            mHTDistanceDeviationFactor(float)\n"
						<< "-m [string]            set output PGM filename to [string]\n"
						<< "--prefix [string]      set ABW rasterfile input and output prefix (without .range)\n"
						<< "--percPrefix [string]  set PERCEPTRON rasterfile input file and output prefix\n"
						<< "--rawpic [string]      set output for pcd and picture of colored points,\n"
						<< "                            works on PointClouds with RGB Data ONLY!\n"
						<< "--pcd [string]         set input pcd file to [string]\n"
						<< "--pclSAC [int]         start PCL RANSAC with [int] maxIterations\n"
						<< "--runtimeFile [string] write time used for segmentation to file [string]\n"
						<< "--depth [int]          start segmentation with HT / CC / RANSAC on octree depth [int]\n";
				exit(0);
			}
		}
	}
}

/*****************************************************************************/

void StructureColoring::initNode(ros::NodeHandle& n)
{
	mNodeHandle = &n;
	//setup subscriber
	if (mParams.mKinect)
		mPointCloud2Subscriber = mNodeHandle->subscribe<sensor_msgs::PointCloud2> (mKinectTopic, 1,
				&StructureColoring::pointCloud2Callback, this);
	if (mParams.mLaser)
		mPointCloudSubscriber = mNodeHandle->subscribe<sensor_msgs::PointCloud> (mParams.mLaserTopic, 1,
				&StructureColoring::pointCloudCallback, this);
	//publisher is set in class RosVisualization / member variable mVis
}

/*****************************************************************************/

void StructureColoring::initParams()
{
	mLastCellSizeWithNormals = mParams.mRho_max;
	mKinectTopic = "/camera/depth_registered/points";
}

/*****************************************************************************/

void StructureColoring::initOctree()
{
	unsigned int numPoints = 512 * 512; //TODO softcode resolution
	if (mParams.mKinect) numPoints = 640 * 480;
	if (mParams.mLaser) numPoints = 2000000;
	if (!mOcTree) mOcTree.reset(new OcTree(mParams.mMinOctreeResolution, mParams.mRho_max, mParams.mPrincipalVarFactor,
			mParams.mMinPointsForNormal, mParams.mCurvThreshold, mParams.mCurv2Threshold, mParams.mSqDistFactor, numPoints));
}

/*****************************************************************************/

void StructureColoring::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	PointCloudPtr pointCloud(new PointCloud());
	PlanePatchVector pointMappingPlanes;//for PlanePatches
	CylinderPatchVector pointMappingCylinders;//for CylinderPatches
	PlanePatches extractedPlanes;
	CylinderPatches extractedCylinders;
	//get input from rosMsg
	filterMsgSetupCloud(*pointCloud, msg, -mParams.mRho_max, mParams.mRho_max, mParams.mKinect, mParams.mWriteRawPic, mParams.mRawPicFilename, mParams.mRawPicCounter++);

	doSegmentation(pointCloud, pointMappingPlanes, extractedPlanes, pointMappingCylinders, extractedCylinders);
}

/*****************************************************************************/

void StructureColoring::doSegmentation(PointCloudPtr pointCloud, PlanePatchVector& pointMapping, PlanePatches& extractedPlanes,
		CylinderPatchVector& pointMappingCylinders, CylinderPatches& extractedCylinders){
	double elapsedTime = 0.0;
	if(mPointCloudMutex.tryLock(100)){
		mPointCloud = pointCloud;
		mPointCloudMutex.unlock();
	}

	if(mParams.mCylinder)
	{
	    initSegmentation(pointMappingCylinders, *pointCloud);
        if (mParams.mPclSAC)
            elapsedTime = segmentCylindersSAC(extractedCylinders, pointMappingCylinders, *mOcTree, pointCloud);
        else
            elapsedTime = segmentCylinders(extractedCylinders, pointMappingCylinders, *mOcTree, pointCloud);
    }
    else
    {
    	initSegmentation(pointMapping, *pointCloud);
        if (mParams.mPclSAC)
            elapsedTime = segmentPlanesSAC(extractedPlanes, pointMapping, pointCloud);
        else
            elapsedTime = segmentPlanes(extractedPlanes, pointMapping, *mOcTree, pointCloud);
    }
	if(mPointCloudMutex.tryLock(100)){
		mPointMapping = pointMapping;
		mPointCloudMutex.unlock();
	}
	if(mPlanePatchesMutex.tryLock(100)){
		if (mParams.mTextures) generateTextures(extractedPlanes, *pointCloud);
		mPlanePatches = extractedPlanes;
		mPlanePatchesMutex.unlock();
	}
	if(mCylinderPatchesMutex.tryLock(100)){
		mCylinderPatches = extractedCylinders;
		mCylinderPatchesMutex.unlock();
	}

	if (std::strcmp(mParams.mRuntimeFilename.c_str(), "") != 0) writeTimeToFile(mParams.mRuntimeFilename, elapsedTime);

	callPublisher(*mOcTree, extractedPlanes, extractedCylinders, pointCloud, pointMapping, pointMappingCylinders);
}

/*****************************************************************************/

void StructureColoring::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg) {
	sensor_msgs::PointCloud2Ptr pc2msg(new sensor_msgs::PointCloud2());
	if (sensor_msgs::convertPointCloudToPointCloud2(*msg.get(), *pc2msg)){
		pcl::PointCloud<pcl::PointXYZ>::Ptr noColorPC(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*pc2msg, *noColorPC);
		PointCloudPtr colorPC(new PointCloud);
		colorPC->points.reserve(noColorPC->points.size());
		for(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator pit = noColorPC->points.begin();
				pit != noColorPC->points.end(); ++pit){
			PointT p;
			p.x = pit->x;
			p.y = pit->y;
			p.z = pit->z;
			p.rgb = 0;
			colorPC->points.push_back(p);
		}
		sensor_msgs::PointCloud2Ptr cpcMsg(new sensor_msgs::PointCloud2());
		pcl::toROSMsg<PointT>(*colorPC, *cpcMsg);
		pointCloud2Callback(cpcMsg);
	}
}

/*****************************************************************************/

void StructureColoring::readPointCloudFromPCD(PointCloud& pointCloud, const std::string& filename, unsigned int& width,
		unsigned int& height, std::vector<unsigned int>& undefPoints, const float& zMax, const float& zMin){
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
	pcl::PCDReader reader;
    if (mParams.mPCDnoRGB)
    {
	    reader.read(filename, *msg);
        pcl::PointCloud<pcl::PointXYZ> my_pointCloud;
        pcl::fromROSMsg(*msg, my_pointCloud);
        pointCloud.points.reserve(my_pointCloud.points.size());
        for (size_t i = 0; i < my_pointCloud.points.size(); ++i)
        {
			PointT p;
			p.rgb = 0x00000000;
            p.x = my_pointCloud.points[i].x;
            p.y = my_pointCloud.points[i].y;
            p.z = my_pointCloud.points[i].z;
            pointCloud.points.push_back(p);
        }
    }
    else
	    reader.read(filename, pointCloud);
	filterCloud(pointCloud, undefPoints, zMax, zMin);
    if (mParams.mPCDnoRGB)
    {
        width = pointCloud.points.size();
        height = 1;
    }
    else
    {
        width = pointCloud.width;
        height =  pointCloud.height;
    }
	pointCloud.width = pointCloud.points.size();
	pointCloud.height = 1;
	ROS_INFO("read PointCloud with %zu points, width = %d, height = %d", pointCloud.points.size(), pointCloud.width, pointCloud.height);
}

/*****************************************************************************/

void StructureColoring::filterCloud(PointCloud& pointCloud, std::vector<unsigned int>& undefPoints, const float& zMax, const float& zMin){
	PointCloud pc2;
	pc2.header = pointCloud.header;
	pc2.points.swap(pointCloud.points);
	pointCloud.points.reserve(pc2.points.size());
	undefPoints.reserve(pc2.points.size());
	for(unsigned int i=0; i<pc2.points.size(); ++i){
		const PointT& p = pc2.points[i];
		if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z < zMax && p.z > zMin)
			pointCloud.points.push_back(p);
		else undefPoints.push_back(i);
	}
}

/*****************************************************************************/

void StructureColoring::getPointsFromFile() {
	PointCloudPtr pointCloud(new PointCloud());
	PlanePatchVector pointMappingPlanes;
	PlanePatches extractedPlanes;
	CylinderPatches extractedCylinders;
	CylinderPatchVector pointMappingCylinders;
	//input (from disc)
	unsigned int width = 512, height = 512;
	std::vector<unsigned int> undefPoints;
    ROS_INFO("reading file: %s", mParams.mInputFilename.c_str());
	if (mParams.mPerceptron)
		readPointCloudFromPERCEPTRONFile(*pointCloud, mParams.mInputFilename, width, height, undefPoints);
	else if (mParams.mLoadPCD)
		readPointCloudFromPCD(*pointCloud, mParams.mInputFilename, width, height, undefPoints, mParams.mRho_max, -mParams.mRho_max);
	else if (mParams.mABW)
		readPointCloudFromABWFile(*pointCloud, mParams.mInputFilename, width, height, undefPoints, mParams.mPGM);
    else
    {
        ROS_ERROR("unknown input type");
        return;
    }

	mOcTree.reset(new OcTree(mParams.mMinOctreeResolution, mParams.mRho_max, mParams.mPrincipalVarFactor, mParams.mMinPointsForNormal,
			mParams.mCurvThreshold, mParams.mCurv2Threshold, mParams.mSqDistFactor, width * height));

	if (mVis) mVis->publishPointCloud(*pointCloud);
	//start segmentation
	doSegmentation(pointCloud, pointMappingPlanes, extractedPlanes, pointMappingCylinders, extractedCylinders);

	if(mParams.mCylinder)
	{
		writeSegments(width, height, undefPoints, pointMappingCylinders, mParams.mOutputFilename, extractedCylinders);
    }
    else
    {
		writeSegments(width, height, undefPoints, pointMappingPlanes, mParams.mOutputFilename, extractedPlanes);
    }

	if(!mParams.mLoadPCD) writeMSNormals(mParams.mNormalOutputFilename, extractedPlanes);
	size_t unsegPoints = 0;
	for(PlanePatchVector::const_iterator ppvit = pointMappingPlanes.begin(); ppvit != pointMappingPlanes.end(); ++ppvit)
	{
		if (!*ppvit) ++unsegPoints;
	}
	ROS_INFO("number of undefined points = %zu, number of unsegmented points = %zu", undefPoints.size(), unsegPoints);
}

/*****************************************************************************/

double StructureColoring::segmentPlanes(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, OcTree& octree,
		const PointCloudPtr& pointCloud) {
    double start = pcl::getTime();

	octree.buildOctree(*pointCloud);
	extractOctreePlanesFromCloud(extractedPlanes, pointMapping, octree, pointCloud);

	//post-processing
	if (mParams.mPostProcessing & 1) {
		mergePlanes(extractedPlanes, pointMapping, *pointCloud);
	}
	if (mParams.mPostProcessing & 2) {
		spreadNodes(octree, extractedPlanes, pointMapping, *pointCloud);
	}
	if (mParams.mPostProcessing & 4)
		mergePlanes(extractedPlanes, pointMapping, *pointCloud);

    double end = pcl::getTime();
	double elapsedTime = end - start;
	ROS_ERROR("response sent %f seconds after receiving pointcloud", elapsedTime);
	return elapsedTime;
}

/*****************************************************************************/

double StructureColoring::segmentCylinders(CylinderPatches& cylinderPatches, CylinderPatchVector& pointMapping, OcTree& octree, const PointCloudPtr& pointCloud){
    double start = pcl::getTime();

	octree.buildOctree(*pointCloud);

	extractOctreeCylindersFromCloud(cylinderPatches, pointMapping, octree, pointCloud);

//    CylinderPatches::iterator it = cylinderPatches.begin();
//    while (it != cylinderPatches.end())
//    {
//        if (!checkCylinder(*it, pointCloud))
//        {
//            it = cylinderPatches.erase(it);
//        }
//        else
//            ++it;
//    }
//    rebuildPointMapping(pointMapping, cylinderPatches);
	ROS_INFO("%zu cylinders left", cylinderPatches.size());

    double end = pcl::getTime();
	double elapsedTime = end - start;
	ROS_ERROR("response sent %f seconds after receiving pointcloud", elapsedTime);
	return elapsedTime;
}

/*****************************************************************************/

void StructureColoring::generateTextures(const PlanePatches& extractedPlanes, const PointCloud& pointCloud){
	for (PlanePatches::const_iterator plane_it = extractedPlanes.begin(); plane_it != extractedPlanes.end(); ++plane_it){
		(*plane_it)->computeRGBTextureMap(pointCloud, mParams.mTexelSizeFactor, mParams.mDilateIterations);
	}
}

/*****************************************************************************/

void StructureColoring::callPublisher(const OcTree& octree, PlanePatches& extractedPlanes, const CylinderPatches& extractedCylinders , const PointCloudPtr pointCloud,
		const PlanePatchVector& pointMapping, const CylinderPatchVector& cylinderPointMapping) {
	CompArea compArea;
	extractedPlanes.sort(compArea);
	if (mVis) {
		mVis->publishOctreeNormalMarker(octree);
		mVis->publishPlaneMarker(extractedPlanes);
		mVis->publishSegmentedPointcloud(pointMapping, extractedPlanes, *pointCloud);
		mVis->publishSegmentedPointcloud(cylinderPointMapping, extractedCylinders, *pointCloud);
		mVis->publishPointCloud(*pointCloud);
		if(mParams.mCylinder){
			mVis->publishCylinderMarker(extractedCylinders);
			mVis->publishCylinderPoints(extractedCylinders, pointCloud);
		}
	}
}

/*****************************************************************************/

void writePicFromPointCloud(const PointCloud& cloudWithFarPoints, const std::string& rawPicFilename){
	cv::Mat pic(cloudWithFarPoints.height, cloudWithFarPoints.width, CV_8UC3);
	for(unsigned int r = 0; r < cloudWithFarPoints.height; ++r){
		for(unsigned int c = 0; c < cloudWithFarPoints.width; ++c){
			pcl::PointXYZRGB pointrgb = cloudWithFarPoints.points[r * cloudWithFarPoints.width + c];
			if(std::isfinite(pointrgb.x) && std::isfinite(pointrgb.y) && std::isfinite(pointrgb.z)){
				float rgbfloat = pointrgb.rgb;
				int rgbint = *reinterpret_cast<int*> (&rgbfloat);
				pic.at<cv::Vec3b>(r, c)[0] = rgbint & 0xff;
				pic.at<cv::Vec3b>(r, c)[1] = (rgbint >> 8) & 0xff;
				pic.at<cv::Vec3b>(r, c)[2] = (rgbint >> 16) & 0xff;
			} else {
				pic.at<cv::Vec3b>(r, c)[0] = 0.f;
				pic.at<cv::Vec3b>(r, c)[1] = 0.f;
				pic.at<cv::Vec3b>(r, c)[2] = 0.f;
			}
		}
	}
	std::string tmpfn(rawPicFilename);
	tmpfn.append(".ras");
	ROS_INFO("%s", tmpfn.c_str());
	cv::imwrite(tmpfn, pic);
}

/*****************************************************************************/

template<typename PointCloudType>
void writePCDFromPointCloud(const PointCloudType& cloudWithFarPoints, const std::string& rawPicFilename){
	pcl::PCDWriter writer;
	std::string tmpfn(rawPicFilename);
	tmpfn.append(".pcd");
	writer.write(tmpfn, cloudWithFarPoints, false);
}

/*****************************************************************************/

void StructureColoring::filterMsgSetupCloud(PointCloud& pointCloud, const sensor_msgs::PointCloud2ConstPtr& pMsg, double zMin, double zMax, bool fromKinect, bool writePic, const std::string& picFilename, unsigned int picCounter) {
    PointCloud::Ptr cloudWithFarPoints(new PointCloud());
	pcl::fromROSMsg(*pMsg, *cloudWithFarPoints);
	if(writePic){
		std::string tmpPicFilename(picFilename);
		char counterChars[10];
		sprintf(counterChars, "%d", picCounter++);
		tmpPicFilename.append(counterChars);
		writePicFromPointCloud(*cloudWithFarPoints, tmpPicFilename);
		writePCDFromPointCloud(*cloudWithFarPoints, tmpPicFilename);
	}

// Create the filtering object
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloudWithFarPoints);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(zMin, zMax);
	pass.filter(pointCloud);
	pointCloud.width = pointCloud.points.size();
	pointCloud.height = 1;
	//	ROS_INFO("cloud: point cloud has %zu points", mPointCloud.points.size());
	if (mParams.mKinect) {
#pragma omp parallel for schedule(dynamic,1)
		for (size_t i = 0; i < pointCloud.points.size(); ++i) {
			PointT& p = pointCloud.points[i];
			PointT tmp = p;
			p.x = tmp.z;
			p.y = -tmp.x;
			p.z = -tmp.y;
		}
	}
}

/*****************************************************************************/

void StructureColoring::filterConnectedNodes(std::vector<NodePointers>& CCInlierNodes, const Plane3D& pp,
		const NodePointers& octreeNodes, float cellSize, GridMap* oldGridMap, NodePointers* notConnectedNodesOutput) {
	float nodeXMin = std::numeric_limits<float>::max();
	float nodeXMax = -std::numeric_limits<float>::max();
	float nodeYMin = std::numeric_limits<float>::max();
	float nodeYMax = -std::numeric_limits<float>::max();
	for(NodePointers::const_iterator npit = octreeNodes.begin(); npit != octreeNodes.end(); ++npit){
		Vec3 tp(pp.transformToXYPlane((*npit)->value_.meanPos));
		if (tp.x() < nodeXMin) nodeXMin = tp.x();
		if (tp.x() > nodeXMax) nodeXMax = tp.x();
		if (tp.y() < nodeYMin) nodeYMin = tp.y();
		if (tp.y() > nodeYMax) nodeYMax = tp.y();
	}
	GridMap gridmap;
	if (oldGridMap == NULL){
		gridmap = GridMap(pp, nodeXMin, nodeXMax, nodeYMin, nodeYMax, cellSize);
	}
	else {
		float xMin = 0.f, xMax = 0.f, yMin = 0.f, yMax = 0.f;
		unsigned int xOff = 0, xAdd = 0, yOff = 0, yAdd = 0;
		oldGridMap->getNewExtremes(xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, nodeXMin, nodeXMax, nodeYMin, nodeYMax, cellSize);
		gridmap = GridMap(*oldGridMap, xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, cellSize);
	}
	//register OcTreeNodes with grid
	gridmap.populate(octreeNodes);

	if (!oldGridMap)
    {   // if we constructed a new grid, startPos must be initialized:
	    gridmap.calculateStartPos();
    }
	gridmap.getConnectedComponentsAndNotConnectedNodes(CCInlierNodes, octreeNodes, cellSize, mParams.mMinOctreeNodes,
			mParams.mMinNodesInCC, notConnectedNodesOutput);
}

/*****************************************************************************/

void StructureColoring::estimatePlane(const pclPointIndices::Ptr& inliers, PlanePatchPtr& planePatch, const PointCloudPtr& pointCloud,
		const float& sacDist) {
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentation<PointT> seg;
	pclPointIndices newInliers;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_MSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(sacDist);
	seg.setInputCloud(pointCloud);
	seg.setIndices(inliers);
	assert(inliers->indices.size() > 3);
	seg.segment(newInliers, coefficients);

	if (newInliers.indices.size() == 0) {
		planePatch.reset();
	} else {
		Vec3 norm(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
		// RANSAC model Coefficients: ax + by + cz + d = 0
		// my model: n*p - d = 0 with d > 0
		// thus i must switch the distance's sign!
		float distance = -1.f * coefficients.values[3];
		if (distance < 0) {
			distance *= -1.f;
			norm *= -1.f;
		}
		planePatch.reset(new PlanePatch(norm, distance, newInliers.indices, *pointCloud, sacDist));
	}
}

void StructureColoring::estimateCylinder(pclPointIndices& inliers, CylinderPatchPtr& cylinder, unsigned int& numPointsBefRansac, const NodePointers& octreeNodes,
		const OcTree& octree, const PointCloudPtr& pointCloud, const float& sacDist,
		const float& normalDistanceWeight, const float& minRadius, const float& maxRadius) {
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    NormalCloud::Ptr normalCloud(new NormalCloud());
	pclPointIndices partCloudIndices;
	PointCloudPtr partCloud(new PointCloud());

	for(NodePointers::const_iterator np_it = octreeNodes.begin(); np_it != octreeNodes.end(); ++np_it){
		PointIndices inlierPoints;
		getAllPointsFromNodes(inlierPoints, NodePointers(1, *np_it), octree, *pointCloud);
		partCloudIndices.indices.reserve(partCloudIndices.indices.size() + inlierPoints.size());
		copy(inlierPoints.begin(), inlierPoints.end(), back_inserter( partCloudIndices.indices));
	}
	numPointsBefRansac = partCloudIndices.indices.size();

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(pointCloud);
	extract.setIndices(boost::make_shared<pclPointIndices>(partCloudIndices));
	extract.setNegative(false);
	extract.filter(*partCloud);

	normalCloud->points.reserve(partCloud->points.size());
	for(unsigned int p_count = 0; p_count < partCloud->points.size(); ++p_count){
		Vec3 point(partCloud->points[p_count].x, partCloud->points[p_count].y, partCloud->points[p_count].z);
		Vec3 eNormal;
		octree.findFinestNormal(eNormal, point);
		pcl::Normal normal;
		normal.normal_x = eNormal.x();
		normal.normal_y = eNormal.y();
		normal.normal_z = eNormal.z();
		normalCloud->points.push_back(normal);
	}
	assert(partCloud->points.size() == normalCloud->points.size());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(sacDist);
	seg.setNormalDistanceWeight(normalDistanceWeight);
	seg.setRadiusLimits (minRadius, maxRadius);
	seg.setInputCloud(partCloud);
	seg.setInputNormals(normalCloud);
	seg.segment(inliers, coefficients);

	if(inliers.indices.size() == 0){
		ROS_ERROR("RANSAC: no cylinder found, aborting");
		cylinder.reset();
		return;
	}
	Vec3 pointOnAxis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
	Vec3 axisDirection(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
	float radius(coefficients.values[6]);
	const PointIndices& inlierIndices(inliers.indices);
	cylinder.reset(new CylinderPatch(inlierIndices, *partCloud, pointOnAxis, axisDirection, radius, sacDist));
}

void StructureColoring::estimateCylinderNNNormals(CylinderPatchPtr& cylinder,
		const PointCloudPtr& pointCloud, const float& sacDist,
		const float& normalDistanceWeight, const float& minRadius, 
        const float& maxRadius, float searchRadius, const Vec3& initialAxis) {
	pcl::ModelCoefficients coefficients;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    NormalCloud::Ptr normalCloud(new NormalCloud);
    pclPointIndices::Ptr partCloudIndices(new pclPointIndices());
    pclPointIndices::Ptr subsampledPartCloudIndices(new pclPointIndices());
	pclPointIndices subsampledInliers, inliers;
	PointCloudPtr partCloud(new PointCloud());

	PointCloudPtr subsampledPartCloud(new PointCloud());
	PointCloudPtr subsampledCloud(new PointCloud());

	copy(cylinder->getInliers().begin(), cylinder->getInliers().end(), back_inserter(partCloudIndices->indices));

	const float subsamplingLeafSize = 0.5f*0.5f*searchRadius;

	if( subsamplingLeafSize > 2.f*mParams.mMinOctreeResolution ) {

		ROS_ERROR("downsampling");

		pcl::VoxelGrid<PointT> subsampler1, subsampler2;
		subsampler1.setSaveLeafLayout(true);
		subsampler1.setInputCloud( pointCloud );
		subsampler1.setLeafSize( subsamplingLeafSize, subsamplingLeafSize, subsamplingLeafSize );
		subsampler1.setSaveLeafLayout(true);
		subsampler1.filter( *subsampledCloud );
		subsampler1.setSaveLeafLayout(true);

		// subsample cylinder indices..
		// build map from subsampled cylinder indices to original cylinder indices
		std::map< int, std::vector< int > > cylinderSubsamplingMap;
		std::set< int > subsampledPartCloudIndicesSet;
		for( unsigned int i = 0; i < partCloudIndices->indices.size(); i++ ) {
			const PointT& p = pointCloud->points[ partCloudIndices->indices[i] ];
			int subsampledIndex = subsampler1.getCentroidIndexAt( subsampler1.getGridCoordinates( p.x, p.y, p.z ) );
			if( subsampledIndex >= 0 ) {
				subsampledPartCloudIndicesSet.insert( subsampledIndex );
				cylinderSubsamplingMap[subsampledIndex].push_back( partCloudIndices->indices[i] );
			}
		}

		for( std::set<int>::iterator it = subsampledPartCloudIndicesSet.begin(); it != subsampledPartCloudIndicesSet.end(); ++it ) {
			subsampledPartCloudIndices->indices.push_back( *it );
		}

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(subsampledCloud);
		extract.setIndices(subsampledPartCloudIndices);
		extract.setNegative(false);
		extract.filter(*subsampledPartCloud);

		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setIndices(subsampledPartCloudIndices);
		ne.setInputCloud (subsampledCloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (subsampledCloud);
		ne.setSearchMethod (tree);

		// Use all neighbors in a sphere of radius ..
		ne.setRadiusSearch (searchRadius);
		ne.setViewPoint(0.f, 0.f, 0.f);

		// Compute the features
		ne.compute (*normalCloud);

		assert(subsampledPartCloud->points.size() == normalCloud->points.size());

		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		seg.setOptimizeCoefficients(false);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(sacDist);
		seg.setNormalDistanceWeight(normalDistanceWeight);
		seg.setRadiusLimits (minRadius, maxRadius);
		(void)initialAxis;
		seg.setInputCloud(subsampledPartCloud);
		seg.setInputNormals(normalCloud);
		seg.segment(subsampledInliers, coefficients);

		// retrieve cylinder inliers in original point cloud
		PointIndices inlierIndices;

		for( unsigned int i = 0; i< subsampledInliers.indices.size(); i++ ) {
			int subsampledCloudIndex = subsampledPartCloudIndices->indices[ subsampledInliers.indices[i] ];
			if( cylinderSubsamplingMap.find( subsampledCloudIndex ) == cylinderSubsamplingMap.end() ) {
				continue;
			}
			copy(cylinderSubsamplingMap[subsampledCloudIndex].begin(), cylinderSubsamplingMap[subsampledCloudIndex].end(), back_inserter(inlierIndices));
		}

		if(inlierIndices.size() < mParams.mMinPointsInCC){
			ROS_ERROR("RANSAC: no cylinder found, aborting");
			cylinder.reset();
			return;
		}

		Vec3 pointOnAxis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
		Vec3 axisDirection(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
		float radius(coefficients.values[6]);
		cylinder.reset(new CylinderPatch(inlierIndices, *pointCloud, pointOnAxis, axisDirection, radius, sacDist));

	}
	else {

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(pointCloud);
		extract.setIndices(partCloudIndices);
		extract.setNegative(false);
		extract.filter(*partCloud);

		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setIndices(partCloudIndices);
		ne.setInputCloud (pointCloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (pointCloud);
		ne.setSearchMethod (tree);

		// Use all neighbors in a sphere of radius ..
		ne.setRadiusSearch (searchRadius);
		ne.setViewPoint(0.f, 0.f, 0.f);

		ros::Time startTime = ros::Time::now();
		// Compute the features
		ne.compute (*normalCloud);
		ROS_ERROR("ne took %lf", (ros::Time::now()-startTime).toSec());

		assert(partCloud->points.size() == normalCloud->points.size());

		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		seg.setOptimizeCoefficients(false);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(10000);
		//seg.setDistanceThreshold(sacDist*(1-normalDistanceWeight)+mAngleEps*normalDistanceWeight);
		seg.setDistanceThreshold(sacDist);
		seg.setNormalDistanceWeight(normalDistanceWeight);
		seg.setRadiusLimits (minRadius, maxRadius);
		(void)initialAxis;
		//seg.setAxis(initialAxis);  // scheint die Ergebnisse nicht zu verbessern, die Laufzeit sinkt auch nicht
		//seg.setEpsAngle(2.f * mAngleEps); // TODO do not hardcode 2.f
		seg.setInputCloud(partCloud);
		seg.setInputNormals(normalCloud);
		seg.segment(inliers, coefficients);

		if(inliers.indices.size() < mParams.mMinPointsInCC){
			ROS_ERROR("RANSAC: no cylinder found, aborting");
			cylinder.reset();
			return;
		}
		PointIndices inlierIndices;
		inlierIndices.reserve(inliers.indices.size());
		for(PointIndices::const_iterator pit = inliers.indices.begin(); pit != inliers.indices.end(); ++pit){
			inlierIndices.push_back(cylinder->getInliers()[*pit]);
		}
		Vec3 pointOnAxis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
		Vec3 axisDirection(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
		float radius(coefficients.values[6]);
		cylinder.reset(new CylinderPatch(inlierIndices, *pointCloud, pointOnAxis, axisDirection, radius, sacDist));

	}
}

//TODO THIS DOES NOT WORK YET and should not be used!
void StructureColoring::estimateCylinderFromNodes(NodeIndices& inliers, CylinderPatchPtr& cylinder, const NodePointers& octreeNodes,
		const OcTree& octree, const PointCloudPtr& pointCloud, const float& sacDist, const float& normalDistanceWeight,
		const float& minRadius, const float& maxRadius) {
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    NormalCloud::Ptr normalCloud(new NormalCloud());
	pclPointIndices partCloudIndices;
	PointCloudPtr partCloud;

	partCloud->points.reserve(octreeNodes.size());
	normalCloud->points.reserve(octreeNodes.size());
	PointT point;
	pcl::Normal normal;
	for(NodePointers::const_iterator np_it = octreeNodes.begin(); np_it != octreeNodes.end(); ++np_it){
		point.x = (*np_it)->value_.meanPos.x();
		point.y = (*np_it)->value_.meanPos.y();
		point.z = (*np_it)->value_.meanPos.z();
		partCloud->points.push_back(point);
		normal.normal_x = (*np_it)->value_.normal.x();
		normal.normal_y = (*np_it)->value_.normal.y();
		normal.normal_z = (*np_it)->value_.normal.z();
		normalCloud->points.push_back(normal);
	}

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(sacDist);
	seg.setNormalDistanceWeight(0.0f);//normalDistanceWeight);
	seg.setRadiusLimits (minRadius, maxRadius);
	seg.setInputCloud(partCloud);
	seg.setInputNormals(normalCloud);
	seg.segment(partCloudIndices, coefficients);

	if(partCloudIndices.indices.size() == 0){
		ROS_ERROR("RANSAC: no cylinder found, aborting");
		cylinder.reset();
		return;
	}
	Vec3 pointOnAxis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
	Vec3 axisDirection(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
	float radius(coefficients.values[6]);
	PointIndices inlierIndices;
	getAllPointsFromNodes(inlierIndices, partCloudIndices.indices, octreeNodes, octree, *pointCloud);
	cylinder.reset(new CylinderPatch(inlierIndices, *partCloud, pointOnAxis, axisDirection, radius, sacDist));
}

/*****************************************************************************/

void StructureColoring::generateSingleSphereOctreeHTBins(SphereUniformSamplings& houghBins, SphereBinsVec& pointNeighborBins,
		const NodePointers& octreeNodes) {
	houghBins = SphereUniformSamplings(1, SphereUniformSampling(mParams.mPhi_resolution, true));
	pointNeighborBins.clear();
	pointNeighborBins.resize(octreeNodes.size());
	float angleEps = mParams.mAngleEps;
	for (unsigned int node = 0; node < octreeNodes.size(); node++) {
		SphereBins neighborBins;
		if (octreeNodes[node]->value_.stable) {
			float curv = octreeNodes[node]->value_.curv / mParams.mCurvThreshold;
			// TODO Use curv to widen the angle used to compute neighborhood as curv is somehow similar to covariance.
			float weight = octreeNodes[node]->value_.numPoints * (1.f - curv);
			const Vec3& n = octreeNodes[node]->value_.normal;
			std::vector<Pair2ui> neighbors;
			houghBins[0].getAllAngleNeighbors(neighbors, n, angleEps);
			for (unsigned int neighbor = 0; neighbor < neighbors.size(); neighbor++) {
				Vec3 norm = houghBins[0].getNormal(neighbors[neighbor].first, neighbors[neighbor].second);
				assert((norm.x()!=0)||(norm.y()!=0)||(norm.z()!=0));
				SphereBin pb;
				pb.rho = 0;
				pb.i = neighbors[neighbor].first;
				pb.j = neighbors[neighbor].second;
				float scale = std::min(1.f, std::max(0.001f, (fabsf(norm.dot(n)) - fabsf(cos(
						angleEps))) / fabsf(cos(angleEps))));
				houghBins[pb.rho].addPointNoNeighbors(pb.pit, pb.wit, pb.i, pb.j, node, weight * scale);
				neighborBins.push_back(pb);
			}
		}
		pointNeighborBins[node] = neighborBins;
	}
}

/*****************************************************************************/

void StructureColoring::estimatePlaneSingleSphereOctreeHT(pclPointIndices& inliers, Plane3DPtr& planePatch,
		const SphereUniformSamplings& houghBins, const SphereBinsVec& pointNeighborBins, const NodePointers& octreeNodes,
		NodeIndices& inlierNodes, const PointCloud& pointCloud, const OcTree& octree, const unsigned int& depth,
		const PlanePatchVector& pointMapping) {
	unsigned int imax = 0, jmax = 0;
	houghBins[0].getMaximum(imax, jmax);
	NodeIndices nodes;
	houghBins[0].getPoints(nodes, imax, jmax);
	//	ROS_INFO("HT: number nodes in best orientation bin: %zu", nodes.size());

	//get "best" rho/distance:
	float HTDistThresh = getHTDistanceFromDepth(depth, octree);
	TriangleDistributionHistogram hist(-mParams.mRho_max, mParams.mRho_max, HTDistThresh, mParams.mHTDistanceDeviationFactor * HTDistThresh, pointMapping.size());
	Vec3 n(0.f, 0.f, 0.f);
	float sumWeight = 0.f;
	for (unsigned int p = 0; p < nodes.size(); p++) {
		float weight = 0.f;
		const SphereBins& bins = pointNeighborBins[nodes[p]];
		for (SphereBins::const_iterator sbit = bins.begin(); sbit != bins.end(); ++sbit)
			if (sbit->i == imax && sbit->j == jmax)
				weight = *(sbit->wit);
		sumWeight += weight;
		n += octreeNodes[nodes[p]]->value_.normal * weight;
	}
	n /= sumWeight;

	for (unsigned int p = 0; p < nodes.size(); p++) {
		Vec3 pos = octreeNodes[nodes[p]]->value_.meanPos;
		float x = pos.x() * n.x();
		float y = pos.y() * n.y();
		float z = pos.z() * n.z();
		float rho = x + y + z;
		if (rho > mParams.mRho_max)
			ROS_ERROR("HT: Plane out of distance %f(%f)",rho, mParams.mRho_max);
		if (rho < -mParams.mRho_max)
			ROS_ERROR("HT: Plane out of distance %f(%f)", rho, -mParams.mRho_max);
		hist.addDistributedToBins(rho, WeightedIdx(1.f, nodes[p]));
	}
	//get maximum weighted rho/distance
	WeightedIdxVector bestBin;
	float bestRho;
	CompareWeightedIdxVector compareWeightedIdxVector;
	hist.getMaxBin(bestBin, bestRho, compareWeightedIdxVector);

	//get indices of inlier points and nodes
	inliers.indices.clear();
	inliers.indices.reserve(bestBin.nodeIndices().size());//this is not the size it will have later. now it has enough space for all nodes, not for all points!
	inlierNodes.clear();
	inlierNodes.reserve(bestBin.nodeIndices().size());
	for (unsigned int p = 0; p < bestBin.nodeIndices().size(); p++) {
		inlierNodes.push_back(bestBin.nodeIndices()[p]);
		NodePointers all_nodes;
		octree.getAllLeafs(all_nodes, octreeNodes[bestBin.nodeIndices()[p]]);
		assert(octree.checkOctreeNodes(all_nodes));
		assert(octree.checkOctreeNodes(octreeNodes));
		for (unsigned int node = 0; node < all_nodes.size(); node++) {
			const PointIndices& nodepoints = octree.getPointsFromNodeId(all_nodes[node]->value_.id);
			for (PointIndices::const_iterator point_it = nodepoints.begin(); point_it != nodepoints.end(); ++point_it) {
				if ((!pointMapping[*point_it])) {
					inliers.indices.push_back(*point_it);
				}
			}
		}
	}
	if (inliers.indices.size() >= 3) {
		Vec3 normal = houghBins[0].getNormal(imax, jmax);
		float distance = bestRho;
		if (distance < 0) {
			distance *= -1.f;
			normal *= -1.f;
		}
		planePatch.reset(new Plane3D(normal, distance));
	} else {
		planePatch.reset();
		inliers.indices.clear();
	}
}

/*****************************************************************************/
/**inliers - is a vector of (int) indices of a subset of nodes on current depth
 */
void StructureColoring::updateHTBins(const NodeIndices& inliers, SphereUniformSamplings& houghBins, SphereBinsVec& pointNeighborBins) {
	for (NodeIndices::const_iterator in_it = inliers.begin(); in_it != inliers.end(); ++in_it) {
		const SphereBins& neighborBins = pointNeighborBins[*in_it];
		for (SphereBins::const_iterator nb_it = neighborBins.begin(); nb_it != neighborBins.end(); ++nb_it) {
			assert(nb_it->rho < houghBins.size());
			houghBins[nb_it->rho].deletePointNWeightAt(nb_it->pit, nb_it->wit, nb_it->i, nb_it->j);
		}
		pointNeighborBins[*in_it].clear();
	}
}

/*****************************************************************************/

void StructureColoring::generateNeighboringNodePairs(NodePairs& nodePairs, NodePointers& octreeNodes,
		const unsigned int neighborhoodSize, const unsigned int& currentOctreeDepth, const OcTree& octree) {
	for (NodePointers::iterator node_it = octreeNodes.begin(); node_it
			!= octreeNodes.end(); ++node_it) {
		if ((*node_it)->value_.stable) {
			NodePointers neighborNodes;
			//get not yet querried neighboring nodes
			octree.getNeighboringNodes(neighborNodes, *node_it,
					currentOctreeDepth, (float)neighborhoodSize);
			for (NodePointers::const_iterator neighborNode_it =
					neighborNodes.begin(); neighborNode_it
					!= neighborNodes.end(); ++neighborNode_it) {
				if ((*neighborNode_it)->value_.stable) {
					NodePair pair(*node_it, *neighborNode_it, mParams.mAngleEps);
					nodePairs.push_back(pair);
				}
			}
		}
		(*node_it)->value_.nodeQuerried = true;
	}
}

/*****************************************************************************/

void StructureColoring::generateCylinderOrientationHB(
		SphereUniformSampling& cylinderOrientationBins,
		SphereBinsVec& binNeighborhood, const NodePairs& nodePairs) {
	cylinderOrientationBins = SphereUniformSampling(mParams.mPhi_resolution);
	binNeighborhood.clear();
	binNeighborhood.resize(nodePairs.size());
	float angleEps = mParams.mAngleEps;
#pragma omp parallel for schedule(dynamic,1) 
	for (unsigned int i = 0; i < nodePairs.size(); ++i) {
		SphereBins neighborBins;
		const Vec3& n = nodePairs[i].getCross();
		// TODO Use curv to widen the angle used to compute neighborhood as curv is somehow similar to covariance.
		float curv = nodePairs[i].getCrossCurv() / mParams.mCurvThreshold;
		float weight = nodePairs[i].getNumPoints() * (1.f - curv);
		std::vector<Pair2ui> neighbors;
		cylinderOrientationBins.getAllAngleNeighbors(neighbors, n, angleEps);
		for (unsigned int neighbor = 0; neighbor < neighbors.size(); neighbor++) {
			Vec3 norm = cylinderOrientationBins.getNormal(
					neighbors[neighbor].first, neighbors[neighbor].second);
			assert((norm.x()!=0)||(norm.y()!=0)||(norm.z()!=0));
			SphereBin pb;
			pb.rho = 0;
			pb.i = neighbors[neighbor].first;
			pb.j = neighbors[neighbor].second;
			float scale = std::min(1.f, std::max(0.001f, (fabsf(norm.dot(n))
					- fabsf(cos(angleEps))) / fabsf(cos(angleEps))));
#pragma omp critical
			cylinderOrientationBins.addPointNoNeighbors(pb.pit, pb.wit, pb.i,
					pb.j, i, weight * scale);
			neighborBins.push_back(pb);
		}
		binNeighborhood[i] = neighborBins;
	}
}

/*****************************************************************************/

void StructureColoring::estimateCylinderHT(const SphereUniformSampling& cylinderOrientationBins, const NodePairs& nodePairs,
		PairIndices& inlierIndices, CylinderPatchPtr& cylinder, const unsigned int& depth, const OcTree& octree,
		const CylinderPatchVector& pointMapping, const PointCloud& pointCloud, const float& minRadius, const float& maxRadius) {
	unsigned int imax = 0, jmax = 0;
	cylinderOrientationBins.getMaximum(imax, jmax);
	PairIndices pairIndices;
	std::vector<float> weights;
	cylinderOrientationBins.getPoints(pairIndices, imax, jmax, &weights);

	Vec3 cylinderOrientation(0.f, 0.f, 0.f);
	float weightSum = 0.f;
	for(PairIndices::const_iterator pairInd_it = pairIndices.begin(); pairInd_it != pairIndices.end();++pairInd_it){
		cylinderOrientation += nodePairs[*pairInd_it].getCross() * weights[pairInd_it-pairIndices.begin()];
		weightSum += weights[pairInd_it-pairIndices.begin()];
	}
	cylinderOrientation /= weightSum;

	Mat3 transformation;
	float cylinderRotationAngle;
	Vec3 cylinderRotationAxis;
	NodePair2D::get2DTransformation(transformation, cylinderRotationAngle, cylinderRotationAxis, cylinderOrientation);

//	Vec3 zAxis = transformation * cylinderOrientation;
	//1D radius histogram TriangleDistributionHistogram
	unsigned int cylinderBins = mParams.mCylinderBins;
	float radBinSize = (maxRadius - minRadius) / (float)cylinderBins;
	float devFactor = mParams.mRadiusDevFactor;//1,5f;
	TriangleDistributionHistogramWithRemove radiusHistogram(minRadius, maxRadius, radBinSize, devFactor * radBinSize, nodePairs.size());
	NodePairs2D nodePairs2D;

	//for each stable pair with normal (each pair in vector):
	unsigned int radiusPairCounter = 0;
	std::vector<int> nodePairIndexFrom2DLookUpTable;
	float htDist = getHTDistanceFromDepth(depth, octree);
	float sacDist = getSACDistanceFromDepth(depth, octree);
	for(PairIndices::const_iterator pairInd_it = pairIndices.begin(); pairInd_it != pairIndices.end();++pairInd_it){
		//project to 2D and calculate 2D circle mid-point and radius
		NodePair2D np2d(nodePairs[*pairInd_it], transformation, sacDist);
		if (np2d.getRadius() < minRadius || np2d.getRadius() > maxRadius || !np2d.stable()) continue;
		nodePairs2D.push_back(np2d);
		radiusHistogram.addDistributedToBins(np2d.getRadius(), WeightedIdx(1.f, radiusPairCounter++));
		nodePairIndexFrom2DLookUpTable.push_back(*pairInd_it);
	}

	//get maximum weighted radius
	WeightedIdxVector bestRadiusBin;
	float bestRadius;

	WeightedIdxVector bestMidPointBin;
	float bestMidPointx = 0.f, bestMidPointy = 0.f;

	unsigned int failcounter = 0;
	unsigned int triesForOneOrientation = mParams.mTriesOnEachDepth;
	do{
		CompareWeightedIdxVector compareWeightedIdxVector;
		radiusHistogram.getMaxBin(bestRadiusBin, bestRadius, compareWeightedIdxVector);
//		ROS_INFO("got bestRadius (%f) from histogram, %zu pairs were in the bin", bestRadius, bestRadiusBin.nodeIndices().size());
		if (bestRadiusBin.nodeIndices().size() < mParams.mMinNodesInCylinder){
			for(PairIndices::const_iterator pairInd_it = pairIndices.begin(); pairInd_it != pairIndices.end();++pairInd_it){
				inlierIndices.push_back(*pairInd_it);
			}
			cylinder.reset();
			return;
		}
		float midPointBinSizeFactor = mParams.mMidPointBinSizeFactor;//1.f;//0.25f;
		float midPointBinSize = htDist * midPointBinSizeFactor;
		float minMidX = std::numeric_limits<float>::max();
		float minMidY = std::numeric_limits<float>::max();
		float maxMidX = -std::numeric_limits<float>::max();
		float maxMidY = -std::numeric_limits<float>::max();
		for(unsigned int binIndex = 0; binIndex < bestRadiusBin.nodeIndices().size(); ++binIndex){
			assert(binIndex < bestRadiusBin.nodeIndices().size());
			assert((unsigned int)bestRadiusBin.nodeIndices()[binIndex] < nodePairs2D.size());
			const NodePair2D& np2d(nodePairs2D[bestRadiusBin.nodeIndices()[binIndex]]);
			const Eigen::Vector2f& mid = np2d.getCircleMidPoint();
			if(!std::isfinite(mid.x()) || !std::isfinite(mid.y())
					|| mid.x() < -2.f * mParams.mRho_max || mid.x() > 2.f * mParams.mRho_max
					|| mid.y() < -2.f * mParams.mRho_max || mid.y() > 2.f * mParams.mRho_max){
				continue;
			}
			if (minMidX > mid.x()) minMidX = mid.x();
			if (minMidY > mid.y()) minMidY = mid.y();
			if (maxMidX < mid.x()) maxMidX = mid.x();
			if (maxMidY < mid.y()) maxMidY = mid.y();
		}
		TriangleDistributionHistogram2D midPointHistogram(minMidX, maxMidX, minMidY, maxMidY,
				midPointBinSize, devFactor   * midPointBinSize);
		unsigned int midpointPairCounter = 0;
		for(unsigned int binIndex = 0; binIndex < bestRadiusBin.nodeIndices().size(); ++binIndex){
			const NodePair2D& np2d(nodePairs2D[bestRadiusBin.nodeIndices()[binIndex]]);

			const Eigen::Vector2f& mid = np2d.getCircleMidPoint();
			if(!std::isfinite(mid.x()) || !std::isfinite(mid.y())
					|| mid.x() < -2.f * mParams.mRho_max || mid.x() > 2.f * mParams.mRho_max
					|| mid.y() < -2.f * mParams.mRho_max || mid.y() > 2.f * mParams.mRho_max){
				continue;
			}
			midPointHistogram.addDistributedToBins(np2d.getCircleMidPoint().x(), np2d.getCircleMidPoint().y(), WeightedIdx(1.f, binIndex));
			++midpointPairCounter;
		}

		//get maximum weighted mid-point
		midPointHistogram.getMaxBin(bestMidPointBin, bestMidPointx, bestMidPointy, compareWeightedIdxVector);

		if (bestMidPointBin.nodeIndices().size() < mParams.mMinNodesInCylinder){
			if(failcounter < triesForOneOrientation){
//			remove all pairs of radius max bin from radius histogram
			radiusHistogram.removeAllFromBin(bestRadius);
//			re-get max from radius histogram
			} else{
				for(PairIndices::const_iterator pairInd_it = pairIndices.begin(); pairInd_it != pairIndices.end();++pairInd_it){
					inlierIndices.push_back(*pairInd_it);
				}
				cylinder.reset();
				return;
			}
		}
	}while(bestMidPointBin.nodeIndices().size() < mParams.mMinNodesInCylinder && failcounter++ < triesForOneOrientation);

	//get indices of inlier points and nodes
	inlierIndices.clear();
	inlierIndices.reserve(2 * bestMidPointBin.nodeIndices().size());//TODO change size reservation! this is not the size it will have later. now it has enough space for all nodes, not for all points!
	std::set<NodePtr> nodeSet;
	for(NodeIndices::const_iterator idx_it = bestMidPointBin.nodeIndices().begin(); idx_it != bestMidPointBin.nodeIndices().end(); ++idx_it){
		inlierIndices.push_back(nodePairIndexFrom2DLookUpTable[bestRadiusBin.nodeIndices()[*idx_it]]);
		const NodePair& np(nodePairs[nodePairIndexFrom2DLookUpTable[bestRadiusBin.nodeIndices()[*idx_it]]]);
		nodeSet.insert(np.getFirst());
		nodeSet.insert(np.getSecond());
	}

	NodePointers inlierNodes;
	inlierNodes.reserve(nodeSet.size());
	copy(nodeSet.begin(), nodeSet.end(), back_inserter(inlierNodes));
	PointIndices pointIndices;
	getPointsFromNodes(pointIndices, inlierNodes, octree, pointCloud, pointMapping);

	Mat3 invTrans = transformation.inverse();
	Vec3 midPoint2(bestMidPointx, bestMidPointy, 0.f);
	Vec3 midPoint3 = invTrans * midPoint2;

	if (pointIndices.size() >= mParams.mMinPointsInCC) {
		cylinder.reset(new CylinderPatch(pointIndices, pointCloud, midPoint3, cylinderOrientation, bestRadius,
				2.f * getSACDistanceFromDepth(depth, octree)));
	} else {
		cylinder.reset();
	}
}

/*****************************************************************************/

void StructureColoring::updateCylinderHTBins(SphereUniformSampling& cylinderOrientationBins,
		const PairIndices& inlierIndices, SphereBinsVec& binNeighborhood) {
	for (PairIndices::const_iterator in_it = inlierIndices.begin(); in_it != inlierIndices.end(); ++in_it) {
		const SphereBins& neighborBins = binNeighborhood[*in_it];
		for (SphereBins::const_iterator nb_it = neighborBins.begin(); nb_it != neighborBins.end(); ++nb_it) {
			cylinderOrientationBins.deletePointNWeightAt(nb_it->pit, nb_it->wit, nb_it->i, nb_it->j);
		}
		binNeighborhood[*in_it].clear();
	}
}

/*****************************************************************************/

void StructureColoring::getAllPointsFromNodes(PointIndices& pointIndices, const NodePointers& octreeNodes,
		const OcTree& octree, const PointCloud& pointCloud) {
	pointIndices.clear();
	pointIndices.reserve(pointCloud.points.size());
	for (NodePointers::const_iterator node_it = octreeNodes.begin(); node_it != octreeNodes.end(); node_it++) {
		NodePointers leaf_nodes;
		octree.getAllLeafs(leaf_nodes, (*node_it));
		for (NodePointers::const_iterator leaf_it = leaf_nodes.begin(); leaf_it != leaf_nodes.end(); leaf_it++) {
			const PointIndices& points = octree.getPointsFromNodePtr(*leaf_it);
			for (std::vector<int>::const_iterator point_it = points.begin(); point_it != points.end(); point_it++) {
				pointIndices.push_back(*point_it);
			}
		}
	}
}

/*****************************************************************************/

void StructureColoring::getAllPointsFromNodes(PointIndices& pointIndices, const NodeIndices& nodeIndices,
		const NodePointers& octreeNodes, const OcTree& octree, const PointCloud& pointCloud) {
	pointIndices.clear();
	pointIndices.reserve(pointCloud.points.size());
	for (NodeIndices::const_iterator node_it = nodeIndices.begin(); node_it != nodeIndices.end(); node_it++) {
		NodePointers leaf_nodes;
		octree.getAllLeafs(leaf_nodes, octreeNodes[*node_it]);
		for (NodePointers::const_iterator leaf_it = leaf_nodes.begin(); leaf_it != leaf_nodes.end(); leaf_it++) {
			const PointIndices& points = octree.getPointsFromNodePtr(*leaf_it);
			for (std::vector<int>::const_iterator point_it = points.begin(); point_it != points.end(); point_it++) {
				pointIndices.push_back(*point_it);
			}
		}
	}
}

/*****************************************************************************/

void StructureColoring::addNodesToPlane(PlanePatchPtr& plane, const NodePointers& nodes, const OcTree& octree,
		const PointCloud& pointCloud, PlanePatchVector& pointMapping) {
	assert(!nodes.empty());
	PointIndices inlierCandidatePoints, inlierPoints;
	getPointsFromNodes(inlierCandidatePoints, nodes, octree, pointCloud, pointMapping);
	inlierPoints.reserve(inlierCandidatePoints.size());
	float xMin = 0.f, xMax = 0.f, yMin = 0.f, yMax= 0.f;
	unsigned int xOff = 0, xAdd = 0, yOff = 0, yAdd = 0;
	float pointXMin = std::numeric_limits<float>::max();
	float pointXMax = -std::numeric_limits<float>::max();
	float pointYMin = std::numeric_limits<float>::max();
	float pointYMax = -std::numeric_limits<float>::max();
	float cellSize = octree.getCellSizeFromDepth((*(nodes.rbegin()))->depth_);
	for (PointIndices::const_iterator poInIt = inlierCandidatePoints.begin(); poInIt != inlierCandidatePoints.end(); ++poInIt) {
		if (plane->getPlane3D().checkDistance(pointCloud.points[*poInIt].getVector3fMap(), plane->getDistanceThreshold())){
			inlierPoints.push_back(*poInIt);
			pointMapping[*poInIt] = plane; //new from comment below
			Vec3 tPoint(plane->getPlane3D().transformToXYPlane(pointCloud.points[*poInIt].getVector3fMap()));
			if(tPoint.x() < pointXMin) pointXMin = tPoint.x();
			if(tPoint.x() > pointXMax) pointXMax = tPoint.x();
			if(tPoint.y() < pointYMin) pointYMin = tPoint.y();
			if(tPoint.y() > pointYMax) pointYMax = tPoint.y();
		}
	}
	//use Points(!) to update grid:
	plane->grid()->getNewExtremes(xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, pointXMin, pointXMax, pointYMin, pointYMax, cellSize);
	plane->setGrid(new GridMap((*(plane->grid())), xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, cellSize));
	plane->computeOrientedBoundingBox();
	plane->computeBoundingRectangle();
	plane->addPoints(inlierPoints, pointCloud);
	plane->grid()->blindPopulate(inlierPoints, pointCloud);
}

/*****************************************************************************/

void StructureColoring::markNodesAsSegmented(const PlanePatch& pp, const NodePointers& nodes, const OcTree& octree,
		const PointCloud& pointCloud, const float& maxDistance, const float& ratio) {
	for (NodePointers::const_iterator np_it = nodes.begin(); np_it != nodes.end(); ++np_it) {
		PointIndices allPointIndices;
		getAllPointsFromNodes(allPointIndices, NodePointers(1, *np_it), octree, pointCloud);
		unsigned int count = 0;
		for (PointIndices::const_iterator point_it = allPointIndices.begin(); point_it != allPointIndices.end(); ++point_it) {
			if (pp.getPlane3D().checkDistance(pointCloud.points[*point_it].getVector3fMap(), maxDistance))
				++count;
		}
		if (ratio <= (float) count / (float) allPointIndices.size()) {
			(*np_it)->sweepDown(NULL, StructureColoring::markAsSegmented);
		}
	}
}

/*****************************************************************************/

void StructureColoring::assignNodesToPlanes(NodePointers& notAssignedOctreeNodes, const unsigned int& octreeDepth,
		const OcTree& octree, PlanePatches& extractedPlanes, const PointCloud& pointCloud, PlanePatchVector& pointMapping) {
	NodePtrList notAssignedNodePtrList;
	for (NodePointerList::const_iterator node_iter = octree.getNodeListOnDepth(octreeDepth).begin(); node_iter
			!= octree.getNodeListOnDepth(octreeDepth).end(); ++node_iter) {
		if ((*node_iter)->value_.stable){
			if (!((*node_iter)->value_.segmented)){
				notAssignedNodePtrList.push_back(*node_iter);
			}
		}
	}
	if (!notAssignedNodePtrList.empty()) {
		float cellSize = octree.getCellSizeFromDepth(octreeDepth);
		if (cellSize < mLastCellSizeWithNormals)
			mLastCellSizeWithNormals = cellSize;
	}
	//sorting is done in main routine before this method is called
	for (PlanePatches::iterator planes_iter = extractedPlanes.begin(); planes_iter != extractedPlanes.end(); ++planes_iter) {
		NodePointers nodeCandidates;
		nodeCandidates.reserve(notAssignedNodePtrList.size());
		std::vector<NodePtrList::iterator> nodeCandidateIterators;
		nodeCandidateIterators.reserve(notAssignedNodePtrList.size());
		for (NodePtrList::iterator node_iter = notAssignedNodePtrList.begin(); node_iter != notAssignedNodePtrList.end(); ++node_iter) {
			if ((*planes_iter)->getPlane3D().checkDistance((*node_iter)->value_.meanPos, (*planes_iter)->getDistanceThreshold())
					&& (*planes_iter)->getPlane3D().checkNormal((*node_iter)->value_.normal, getAngleEpsFromDepth(octreeDepth,
							octree))) {
				nodeCandidateIterators.push_back(node_iter);
				nodeCandidates.push_back(*node_iter);
			}
		}
		if (nodeCandidates.empty())
			continue;
		NodeIndices inlierIndices;
		(*planes_iter)->grid().get()->checkNodesAgainstGridConnection(inlierIndices, nodeCandidates, octree.getCellSizeFromDepth(octreeDepth),
				std::min(mParams.mMinOctreeNodes, mParams.mMinNodesInCC));
		//erase nodes, that are connected with this plane
		NodePointers inlierNodes;
		inlierNodes.reserve(inlierIndices.size());
		for (NodeIndices::const_iterator in_it = inlierIndices.begin(); in_it != inlierIndices.end(); ++in_it) {
			assert((unsigned int)*in_it < nodeCandidates.size());
			inlierNodes.push_back(nodeCandidates[*in_it]);
			notAssignedNodePtrList.erase(nodeCandidateIterators[*in_it]);
		}
		if (!inlierNodes.empty()) {
			//addPoints to this plane and update mSegmented
			addNodesToPlane(*planes_iter, inlierNodes, octree, pointCloud, pointMapping);
			markNodesAsSegmented(**planes_iter, inlierNodes, octree, pointCloud, getSACDistanceFromDepth(octreeDepth, octree),
					mParams.mNodeSegmentedRatio);
		}
	}
	notAssignedOctreeNodes.resize(notAssignedNodePtrList.size());
	unsigned int i = 0;
	for (NodePtrList::const_iterator noas_iter = notAssignedNodePtrList.begin(); noas_iter != notAssignedNodePtrList.end(); ++noas_iter) {
		notAssignedOctreeNodes[i++] = *noas_iter;
	}
}

/*****************************************************************************/
/*
 void StructureColoring::extractOctreePlanesFromCloud(std::vector<SphereUniformSampling>& planeHTBins)
 {
 für jede Octree-Ebene:
 Knoten zu bisherigen Ebenen hinzufügen
 Zusammenhang bestimmen -> Ebenen zerteilen
 Restliche (gute) Knoten zu neuen Ebenen hinzufügen
 Punkte betrachten für RANSAC
 schlechte Knoten ignorieren


 --
 neue Ebene:
 Hough-Transformation
 ConnectedComponents
 RANSAC
 }
 */
void StructureColoring::extractOctreePlanesFromCloud(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
		const OcTree& octree, const PointCloudPtr& pointCloud){
	//ROS_INFO("octree depth is: %d", mOctreeDepth);
	NodePointers octreeNodes;
	assert(octree.checkSamplingMap());
	assert(octree.checkGetAllLeafs());

	PointCloud histogramPC;
	unsigned int maxDepth = octree.getMaxDepth();
	unsigned int startDepth = 0;
	if (mParams.mDebugSteps >= 0 && (unsigned int) mParams.mDebugSteps <= maxDepth * 2)
		maxDepth = mParams.mDebugSteps / 2;
	if(mParams.mOnlyDepth){
		startDepth = mParams.mOnlyDepth;
		maxDepth = mParams.mOnlyDepth;
	}
	for (unsigned int depth = startDepth; depth <= maxDepth; depth++) {
//ROS_INFO("assign points to planes on depth %d", depth);
		//	for (unsigned int depth = 0; depth <= 5; depth++){//for debug use and picture taking of single steps
		octreeNodes.clear();
		CompArea compArea;
		extractedPlanes.sort(compArea);
		assignNodesToPlanes(octreeNodes, depth, octree, extractedPlanes, *pointCloud, pointMapping);
//ROS_INFO("points assigned to planes on depth %d", depth);
		if (mParams.mOnlyDepth) ROS_INFO("cellsize %f on depth %i", octree.getCellSizeFromDepth(depth), depth);
		if ((mParams.mDebugSteps < 0 || depth != maxDepth || mParams.mDebugSteps & 1) && octreeNodes.size() > mParams.mMinNodesInCC) {
			float sacDist = getSACDistanceFromDepth(depth, octree);

			SphereUniformSamplings planeHTBins;
			SphereBinsVec pointNeighborhood;
			generateSingleSphereOctreeHTBins(planeHTBins, pointNeighborhood, octreeNodes);
//			if(depth == 6)
			if (mVis) {
//				mVis->publishHistogramMarker(planeHTBins[0], .05f, Vec3(0.f, 1.f - 0.125f * depth, 0.f));
				PointCloud tmpPointCloud;
				planeHTBins[0].getHistogramAsPointCloud(tmpPointCloud, .1f, Vec3(.5f, 0.5f, 0.f));
				histogramPC += tmpPointCloud;
			}
			NodeIndices inlierNodes;
			unsigned int failcounter = 0;
			unsigned int numberPoints = 0;
			////DEBUG output
			unsigned int HTCounter = 0;
			////DEBUG output ^^
			do {
				Plane3DPtr pp;
				inlierNodes.clear();
				pclPointIndices planeInliers;
				estimatePlaneSingleSphereOctreeHT(planeInliers, pp, planeHTBins, pointNeighborhood, octreeNodes, inlierNodes, *pointCloud, octree, depth, pointMapping);
				//ROS_INFO("PlanePatch normal (%f, %f, %f) and distance (%f) after HT", pp.getPlaneNormal().x(), pp.getPlaneNormal().y(), pp.getPlaneNormal().z(), pp.getPlaneDistanceToOrigin());
				////DEBUG output
				Vec3 color(Vec3::Zero());
				if (mParams.mDebugSteps != -1 && depth != maxDepth && mParams.mDebugSteps & 1) {
					PointIndices HTPointIndices;
					getAllPointsFromNodes(HTPointIndices, getNodesFromIndices(inlierNodes, octreeNodes), octree, *pointCloud);
					char app[255];
					sprintf(app, "HTPlane HT(%d) DEBUG depth(%d)", HTCounter, depth);
					float rgb[3];
					RosVisualization::getColorByIndex(rgb, HTCounter, failcounter + 10);
					color = Vec3(rgb[0], rgb[1], rgb[2]);
					if (mVis) mVis->publishPoints(getPointsFromIndices(HTPointIndices, *pointCloud), std::string(app), color);
				}
				////DEBUG output ^
				updateHTBins(inlierNodes, planeHTBins, pointNeighborhood);
				if (inlierNodes.size() < mParams.mMinOctreeNodes) continue;
//ROS_INFO("plane has %zu points before computing CCs", planeInliers.indices.size());
				if ((pp) && (planeInliers.indices.size() >= mParams.mMinPointsInCC)) {
					std::vector<NodePointers> CCInliers;
					NodePointers inlierOctreeNodes;
					for (NodeIndices::const_iterator in_it = inlierNodes.begin(); in_it != inlierNodes.end(); in_it++) {
						inlierOctreeNodes.push_back(octreeNodes[*in_it]);
					}
					filterConnectedNodes(CCInliers, *pp, inlierOctreeNodes, octree.getCellSizeFromDepth(depth));
//ROS_INFO("plane has %zu connected Components", CCInliers.size());
					for (unsigned int cc = 0; cc < CCInliers.size(); cc++) {
						//ROS_INFO("PlanePatch normal (%f, %f, %f) and distance (%f) after CCFiltering", planePatchesVec[cc].getPlaneNormal().x(), planePatchesVec[cc].getPlaneNormal().y(), planePatchesVec[cc].getPlaneNormal().z(), planePatchesVec[cc].getPlaneDistanceToOrigin());
                        pclPointIndices::Ptr pinliers = boost::make_shared<pclPointIndices>(); // create empty PointIndices and shared_ptr to it
						getPointsFromNodes(pinliers->indices, CCInliers[cc], octree, *pointCloud, pointMapping);
						////DEBUG output
						if (mParams.mDebugSteps != -1 && depth != maxDepth && mParams.mDebugSteps & 1) {
							char nameApp[255];
							sprintf(nameApp, "CCPlane CC(%d) DEBUG HT(%d) depth(%d)", cc, HTCounter, depth);
							//							float rgb[3];
							//							RosVisualization::getColorByIndex(rgb, cc, CCInliers.size());
							//							Vec3 color2(rgb[0], rgb[1], rgb[2]);
							if (mVis) mVis->publishPoints(getPointsFromIndices(pinliers->indices, *pointCloud), std::string(nameApp),
									color);
						}
						PointIndices CCIndices = pinliers->indices;
						////DEBUG output ^
						size_t sizeOfInliers = pinliers->indices.size();
						if (sizeOfInliers >= mParams.mMinPointsInCC) {
							PlanePatchPtr pp2;
							//							pp2.reset(new PlanePatch(pp.getPlaneNormal(), pp.getPlaneDistanceToOrigin(), pinliers.indices, mPointCloud));
							//ROS_INFO("pp2 normal (%f, %f, %f) and distance (%f) ", pp2->getPlaneNormal().x(), pp2->getPlaneNormal().y(), pp2->getPlaneNormal().z(), pp2->getPlaneDistanceToOrigin());
                            if (mParams.mNoRansacStep){
                            	const PointIndices& pp1Inliers(pinliers->indices);
                            	PointIndices pp2Inliers;
                            	pp2Inliers.reserve(pp1Inliers.size());
                            	for(PointIndices::const_iterator pis_it = pp1Inliers.begin(); pis_it != pp1Inliers.end(); ++pis_it){
                            		Vec3 pis_it_point(pointCloud->points[*pis_it].x, pointCloud->points[*pis_it].y, pointCloud->points[*pis_it].z);
									if(pp->checkDistance(pis_it_point, getHTDistanceFromDepth(depth, octree)))
										pp2Inliers.push_back(*pis_it);
                            	}
                            	//generate new pp2!
                            	if ((float)pp2Inliers.size() / (float)sizeOfInliers >= mParams.mSACOutlierRatioThreshold){
                            		pp2.reset(new PlanePatch(pp->getPlaneNormal(), pp->getPlaneDistanceToOrigin(), pp2Inliers, *pointCloud, getHTDistanceFromDepth(depth, octree)));
//ROS_INFO("pp2 normal (%f, %f, %f) and distance (%f) ", pp2->getPlane3D().getPlaneNormal().x(), pp2->getPlane3D().getPlaneNormal().y(), pp2->getPlane3D().getPlaneNormal().z(), pp2->getPlane3D().getPlaneDistanceToOrigin());
                            	}
                            } else
							    estimatePlane(pinliers, pp2, pointCloud, sacDist);
							if (pp2 && (pp->checkNormal(pp2->getPlane3D().getPlaneNormal(), 2.f * getAngleEpsFromDepth(depth, octree)))
									&& (fabsf(pp->getPlaneDistanceToOrigin() - pp2->getPlane3D().getPlaneDistanceToOrigin()) < 2.f
											* getHTDistanceFromDepth(depth, octree)) && (pp2->getInliers().size()
									>= mParams.mMinPointsInCC) && (((float) pp2->getInliers().size()) / ((float) sizeOfInliers)
									>= mParams.mSACOutlierRatioThreshold)) {
								markNodesAsSegmented(*pp2, CCInliers[cc], octree, *pointCloud, getSACDistanceFromDepth(depth, octree),
										mParams.mNodeSegmentedRatio);
								float cellSize = octree.getCellSizeFromDepth(depth);
//ROS_INFO("new grid with blind spots from %zu points", pp2->getInliers().size());
								pp2->newGrid(cellSize);
								pp2->grid()->blindPopulate(pp2->getInliers(), *pointCloud);
								pp2->grid()->calculateStartPos();
//pp2->grid()->print();
//								if (mDebugSteps < 0 || depth <= maxDepth) {
									extractedPlanes.push_front(pp2);
//								}
								numberPoints += pp2->getInliers().size();
								////DEBUG output
								if (mParams.mDebugSteps != -1 && depth != maxDepth && mParams.mDebugSteps & 1) {
									char nameApp[255];
									sprintf(nameApp, "Plane b.R. CC(%d) DEBUG HT(%d) depth(%d)", cc, HTCounter, depth);
									PointIndices pointIndices;
									sort(CCIndices.begin(), CCIndices.end());
									PointIndices planePointIndices = pp2->getInliers();
									sort(planePointIndices.begin(), planePointIndices.end());
									set_difference(CCIndices.begin(), CCIndices.end(), planePointIndices.begin(),
											planePointIndices.end(), std::back_inserter(pointIndices));
									Vec3 color2 = Vec3(1.f, 0.f, 0.f);
									if (mVis) mVis->publishPoints(getPointsFromIndices(pointIndices, *pointCloud),
											std::string(nameApp), color2);

									char name[255];
									sprintf(name, "RANSACPlane CC(%d) DEBUG HT(%d) depth(%d)", cc, HTCounter, depth);
									color2 = Vec3(0.f, 1.f, 0.1f);
									if (mVis) mVis->publishPoints(getPointsFromIndices(pp2->getInliers(), *pointCloud), std::string(
											name), color2);
								}
								////DEBUG output ^
//								if (mDebugSteps < 0 || depth <= maxDepth) {
									for (PointIndices::const_iterator in_it = pp2->getInliers().begin(); in_it
											!= pp2->getInliers().end(); in_it++) {
										pointMapping[*in_it] = pp2;
									}
//								}
//ROS_ERROR("plane: added plane patch (%zu with %zu points) to collection. normal = (%f, %f, %f), distance = %f", extractedPlanes.size()-1 , pp2->getInliers().size(), pp2->getPlane3D().getPlaneNormal().x(), pp2->getPlane3D().getPlaneNormal().y(), pp2->getPlane3D().getPlaneNormal().z(), pp2->getPlane3D().getPlaneDistanceToOrigin());
							}
						}
					}
				}
				////DEBUG output
				HTCounter++;
				////DEBUG output ^
			} while (((inlierNodes.size() >= mParams.mMinOctreeNodes) && (numberPoints > mParams.mMinPointsInCC)) || (failcounter++
					< mParams.mTriesOnEachDepth));
			//			break;
		}//uncomment to speed-up as above
	}
	if (mVis) {
		mVis->publishHTBinCloud(histogramPC);
	}
}

/*****************************************************************************/

void StructureColoring::assignNodesToCylinders(NodePointers& octreeNodes, CylinderPatches& extractedCylinders,
		CylinderPatchVector& pointMapping, const OcTree& octree, const PointCloud& pointCloud){
	typedef std::list<NodePtr> NodeList;
	typedef std::vector<NodeList::iterator> NodeListIterators;
	NodeList nodeList;
    std::copy(octreeNodes.begin(), octreeNodes.end(), std::back_inserter(nodeList));
	for(CylinderPatches::iterator cit = extractedCylinders.begin(); cit != extractedCylinders.end(); ++cit){
		NodePointers nodesToAdd;
		NodeListIterators nodesToBeErased;
		for(NodeList::iterator node_it = nodeList.begin(); node_it != nodeList.end(); ++node_it){
			if((*cit)->checkDistance((*node_it)->value_.meanPos)
					&& (*cit)->checkNormal((*node_it)->value_.normal, (*node_it)->value_.meanPos, mParams.mAngleEps)
					&& (*cit)->checkHeight((*node_it)->value_.meanPos, mParams.mCylinderHeightDev))
            {
				nodesToAdd.push_back(*node_it);
				nodesToBeErased.push_back(node_it);
			}
		}
		for(NodeListIterators::const_iterator nit_it = nodesToBeErased.begin(); nit_it != nodesToBeErased.end(); ++nit_it){
			nodeList.erase(*nit_it);
		}
		PointIndices pointIndicesToAdd;
		getPointsFromNodes(pointIndicesToAdd, nodesToAdd, octree, pointCloud, pointMapping);
		updatePointMapping(pointMapping, pointIndicesToAdd, *cit);
		(*cit)->addPoints(pointIndicesToAdd, pointCloud);
	}
	octreeNodes.clear();
    std::copy(nodeList.begin(), nodeList.end(), std::back_inserter(octreeNodes));
}

/*****************************************************************************/

void StructureColoring::extractOctreeCylindersFromCloud(CylinderPatches& extractedCylinders, CylinderPatchVector& pointMapping,
		const OcTree& octree, const PointCloudPtr& pointCloud){
	NodePointers octreeNodes;
	unsigned int startDepth = 0;
	unsigned int maxDepth = octree.getMaxDepth();
	unsigned int cylinderPairNeighborhoodSize = mParams.mCylinderPairNeighbors;
	if (mParams.mDebugSteps >= 0 && (unsigned int) mParams.mDebugSteps <= maxDepth * 2)
		maxDepth = mParams.mDebugSteps / 2;
	PointCloud histogramPC;
	if(mParams.mOnlyDepth){
		startDepth = mParams.mOnlyDepth;
		maxDepth = mParams.mOnlyDepth;
	}
	for (unsigned int depth = startDepth; depth <= maxDepth; depth++) 
    {
		float sacDist = getSACDistanceFromDepth(depth, octree);
		float minRadius = 0.f, maxRadius = 0.f;
		getMinMaxRadiusFromDepth(minRadius, maxRadius, octree, depth);
		ROS_INFO("minRadius = %f, maxRadius = %f", minRadius, maxRadius);
		SphereUniformSampling cylinderOrientationBins(mParams.mPhi_resolution);
		SphereBinsVec binNeighborhood;
		NodePairs nodePairs;
		octreeNodes.clear();
		octreeNodes = octree.getNodesOnDepth(depth);
        size_t nrNodesOnDepth = octreeNodes.size();
		if(octreeNodes.size() < mParams.mMinNodesInCylinder) continue;
		assignNodesToCylinders(octreeNodes, extractedCylinders, pointMapping, octree, *pointCloud);
		if(octreeNodes.size() < mParams.mMinNodesInCylinder) continue;
		generateNeighboringNodePairs(nodePairs, octreeNodes, cylinderPairNeighborhoodSize, depth, octree);
		if(mVis){
			mVis->publishPairMarker(nodePairs);
		}
		ROS_INFO("depth %d: %zu cylinders, %zu octreeNodes, %zu not assigned, %zu node pairs", depth, extractedCylinders.size(), nrNodesOnDepth, octreeNodes.size(), nodePairs.size());
		if (nodePairs.size() < mParams.mMinNodesInCylinder) continue;
		generateCylinderOrientationHB(cylinderOrientationBins, binNeighborhood, nodePairs);
//		ROS_INFO("cylinder orientation histogram generated, binNeighborhood has %zu entries", binNeighborhood.size());

		if (mVis) {
			PointCloud tmpPointCloud;
			cylinderOrientationBins.getHistogramAsPointCloud(tmpPointCloud, .05f, Vec3(-.5f, 1.f - 0.125f * depth, 0.f));
			histogramPC += tmpPointCloud;
		}

		unsigned int failcounter = 0;
		unsigned int numberPoints;
//		float htDist = getHTDistanceFromDepth(depth, octree);

		PairIndices pairIndices;
		do{
//			ROS_ERROR("starting new cylinder HT (max orientation extraction), %d fails yet", failcounter);
			numberPoints = 0;
			CylinderPatchPtr cylinder;
			NodePointers nodePointers;
			pairIndices.clear();
			estimateCylinderHT(cylinderOrientationBins, nodePairs, pairIndices, cylinder, depth, octree, pointMapping, *pointCloud, minRadius, maxRadius);
			updateCylinderHTBins(cylinderOrientationBins, pairIndices, binNeighborhood);
			if(!cylinder || pairIndices.size() < mParams.mMinOctreeNodes || (!checkCylinderDimensions(cylinder))){
				continue;
			}
			Vec3 htAxisDirection(cylinder->getAxisDirection());
			Vec3 htMidPoint(cylinder->getMidPoint());
			float htRadius(cylinder->getRadius());
			float cellSize(octree.getCellSizeFromDepth(depth));
			//filterCylinderConnectedNodes();
			unsigned int htNumPoints = cylinder->getInliers().size();
            if (!mParams.mNoRansacStep)
            {
                estimateCylinderNNNormals(cylinder, pointCloud, sacDist, mParams.mNormalDistanceWeight,
                		0.001, 10.0,
                		1.0*cellSize, htAxisDirection);
                if (!cylinder){
                    ROS_WARN("estimateCylinderNNNormals() removed cylinder");
                    continue;
                }
                //some check helper
                float cosAngle = htAxisDirection.dot(cylinder->getAxisDirection());
                //ransac result similar to ht result check

                if(fabsf(cosAngle) < cosf(2.f * mParams.mAngleEps)) {
                    ROS_WARN("results of HT and RANSAC were too different (angle) %f %f", fabsf(cosAngle), cosf(2.f * mParams.mAngleEps) );
                    continue;
                }

                if(cylinder->getRadius() / htRadius > 2 || htRadius / cylinder->getRadius() > 2) {
                    ROS_WARN("results of HT and RANSAC were too different (radius) %f", cylinder->getRadius() / htRadius );
                    continue;
                }

                if((cylinder->getInliers().size() < htNumPoints * mParams.mSACOutlierRatioThreshold)){
                    ROS_WARN("results of HT and RANSAC were too different (num inliers)");
                    continue;
                }
            }
            if (!checkCylinder(cylinder, pointCloud)){
            	ROS_WARN("not enough area on cylinder");
            	continue;
            }
			updatePointMapping(pointMapping, cylinder->getInliers(), cylinder);
			extractedCylinders.push_back(cylinder);
			numberPoints = cylinder->getInliers().size();
		} while (((pairIndices.size() >= mParams.mMinOctreeNodes) && (numberPoints > mParams.mMinPointsInCC)) || (failcounter++ < mParams.mTriesOnEachDepth));
		ROS_INFO("no more cylinders on depth %d", depth);
	}
	ROS_INFO("cylinder HT completed for all depths");
	if(mVis){
		mVis->publishHTBinCloud(histogramPC);
	}
}

/*****************************************************************************/

void StructureColoring::getNodesFromPairs(NodePointers& nodePointers, const PairIndices& pairIndices, const NodePairs& nodePairs){
	nodePointers.reserve(pairIndices.size() * 2);
	for(PairIndices::const_iterator pair_it = pairIndices.begin(); pair_it != pairIndices.end(); ++pair_it){
		nodePointers.push_back(nodePairs[*pair_it].getFirst());
		nodePointers.push_back(nodePairs[*pair_it].getSecond());
	}
}

/*****************************************************************************/

void StructureColoring::determineCandidatePlanes(NodeIndexToPlanePatchPointers& nodeToPlaneCandidates,
		const NodePointers& octreeNodes, PlanePatches& extractedPlanes) {
#pragma omp parallel for schedule(dynamic,1)
	for (size_t nodeCount = 0; nodeCount < octreeNodes.size(); ++nodeCount) {
		for (PlanePatches::iterator planes_iter = extractedPlanes.begin(); planes_iter != extractedPlanes.end(); ++planes_iter) {
			if ((*planes_iter)->getPlane3D().checkDistance(octreeNodes[nodeCount]->value_.meanPos,
					(*planes_iter)->getDistanceThreshold()))
			{
				const Vec3& nodeCoG = octreeNodes[nodeCount]->value_.meanPos;
				if (((*planes_iter)->distanceToOBB(nodeCoG) < std::sqrt((*planes_iter)->getArea()) * mParams.mNodeToBBDistance))
				{
					if((*planes_iter)->checkPointConnection(nodeCoG, mParams.mConnectionNeighbors))
					{
						nodeToPlaneCandidates[nodeCount].push_back(*planes_iter);
					}
				}
			}
		}
	}
}

/*****************************************************************************/

void StructureColoring::determineCandidatePlanes(PatchToNodeIndicesMap& outIndicesMap,
		PatchToNodePointersMap& outPointersMap, const NodePointers& octreeNodes, PlanePatches& extractedPlanes) {
	// reserve memory:
	for (PlanePatches::iterator planes_iter = extractedPlanes.begin(); planes_iter != extractedPlanes.end(); ++planes_iter) {
		unsigned int possSize = std::min(octreeNodes.size(), (*planes_iter)->getInliers().size());
		outIndicesMap[*planes_iter].reserve(possSize);
		outPointersMap[*planes_iter].reserve(possSize);
	}
	// determine candidates:
	for (size_t nodeCount = 0; nodeCount < octreeNodes.size(); ++nodeCount) {
		for (PlanePatches::iterator planes_iter = extractedPlanes.begin(); planes_iter != extractedPlanes.end(); ++planes_iter) {
			if ((*planes_iter)->getPlane3D().checkDistance(octreeNodes[nodeCount]->value_.meanPos,
					(*planes_iter)->getDistanceThreshold())) {
				outIndicesMap[*planes_iter].push_back(nodeCount);
				outPointersMap[*planes_iter].push_back(octreeNodes[nodeCount]);
			}
		}
	}
}

/*****************************************************************************/

void StructureColoring::refineAssignmentWithCCs(NodeIndexToPlanePatchPointers& nodeToPlaneCandidates,
		const PatchToNodeIndicesMap& nodeIndicesMap, const PatchToNodePointersMap& nodePointersMap,
		const NodePointers& octreeNodes, PlanePatches& extractedPlanes) {
	for (PlanePatches::iterator planes_iter = extractedPlanes.begin(); planes_iter != extractedPlanes.end(); ++planes_iter) {
		const NodePointers& nodePointers = nodePointersMap.find(*planes_iter)->second;
		size_t nodeCount = 0;
		for (NodePointers::const_iterator nit = nodePointers.begin(); nit != nodePointers.end(); ++nit) {
			const Vec3& nodeCoG = (*nit)->value_.meanPos;
			if ((*planes_iter)->distanceToOBB(nodeCoG) < std::sqrt((*planes_iter)->getArea()) * mParams.mNodeToBBDistance
					&& (*planes_iter)->checkPointConnection(nodeCoG, mParams.mConnectionNeighbors)) {
				const NodeIndices& nodeIndices = nodeIndicesMap.find(*planes_iter)->second;
				nodeToPlaneCandidates[nodeIndices[nodeCount]].push_back(*planes_iter);
			}
			++nodeCount;
		}
	}
}

/*****************************************************************************/

void StructureColoring::calculateMiddlePlane(Plane3D& outParams, const PlanePatch& firstPPP,
		const PlanePatch& secondPPP) {
	Vec3 viewport(Vec3::Zero());
	Vec3 p1Norm = firstPPP.getPlane3D().getPlaneNormal();
	flipToViewport(p1Norm, firstPPP.getPlaneCoG() - viewport);
	Vec3 p2Norm = secondPPP.getPlane3D().getPlaneNormal();
	flipToViewport(p2Norm, secondPPP.getPlaneCoG() - viewport);
	Vec3 tang1(p2Norm.cross(p1Norm));
	Vec3 tang2((p1Norm + p2Norm));
	tang2.normalize();
	Vec3 normal = tang1.cross(tang2);
	normal.normalize();
	Vec3 v = tang1.cross(p1Norm);
	Vec3 pointOnPlane(firstPPP.getPlane3D().getOrthogonalProjectionOntoPlane(firstPPP.getPlaneCoG()));
	float lambda = (secondPPP.getPlane3D().getPlaneDistanceToOrigin() - pointOnPlane.dot(secondPPP.getPlane3D().getPlaneNormal()))
			/ secondPPP.getPlane3D().getPlaneNormal().dot(v);
	Vec3 pointOnLineOfIntersection(pointOnPlane + lambda * v);
	if (pointOnLineOfIntersection.dot(normal) < 0) {
		normal *= -1.f;
	}
	outParams.setPlaneNormal(normal);
	outParams.setPlaneDistanceToOrigin(pointOnLineOfIntersection.dot(normal));
}

/*****************************************************************************/

void StructureColoring::refineEdges(PatchToNodePointersMap& finalPlaneNodeAssignmentMap,
		PatchToPointIndicesMap& finalPlanePointAssignmentMap, const NodeIndexToPlanePatchPointers& nodeToPlaneCandidates,
		const NodePointers& octreeNodes, const OcTree& octree, const PointCloud& pointCloud) {
	typedef std::pair<PlanePatchPtr, PlanePatchPtr> PlanePatchPointerPair;
	typedef std::map<PlanePatchPointerPair, Plane3D, CompPlanePtrPair> MiddlePlaneMap;
	typedef std::map<PlanePatchPointerPair, PointIndices, CompPlanePtrPair> MiddlePlaneIndicesMap;
	MiddlePlaneMap middlePlaneMap;
	MiddlePlaneIndicesMap middlePlaneIndicesMap;
	size_t nodeIndex = 0;
	for (NodeIndexToPlanePatchPointers::const_iterator node_idx_it = nodeToPlaneCandidates.begin(); node_idx_it
			!= nodeToPlaneCandidates.end(); ++node_idx_it) {
		if (node_idx_it->size() > 1) {//node has more than one possible Plane
			PointIndices pointIndices;
			getAllPointsFromNodes(pointIndices, NodePointers(1, octreeNodes[nodeIndex]), octree, pointCloud);
			for (PointIndices::const_iterator point_it = pointIndices.begin(); point_it != pointIndices.end(); ++point_it) {
				PlanePatchPtr firstPPP, secondPPP;
				float firstDist = std::numeric_limits<float>::max(), secondDist = std::numeric_limits<float>::max();
				for (PlanePatchVector::const_iterator pp_it = node_idx_it->begin(); pp_it != node_idx_it->end(); ++pp_it) {
					float currDist = (*pp_it)->getPlane3D().distance(octreeNodes[nodeIndex]->value_.meanPos);
					if (currDist <= firstDist) {
						secondDist = firstDist;
						secondPPP = firstPPP;
						firstPPP = *pp_it;
						firstDist = currDist;
					} else if (currDist <= secondDist) {
						secondPPP = *pp_it;
						secondDist = currDist;
					}
				}
				//look for middle plane in map
				bool notInList = true;
				PlanePatchPointerPair pppp(std::make_pair(firstPPP, secondPPP));
				PlanePatchPointerPair ppppSwap(std::make_pair(secondPPP, firstPPP));
				MiddlePlaneMap::iterator mpm_it = middlePlaneMap.find(pppp);
				MiddlePlaneMap::iterator mpmSwap_it = middlePlaneMap.find(ppppSwap);
				if (mpm_it != middlePlaneMap.end()) {
					notInList = false;
				} else if (mpmSwap_it != middlePlaneMap.end()) {
					notInList = false;
					mpm_it = mpmSwap_it;
					pppp = ppppSwap;
				}
				Plane3D planeParams;
				if (notInList) {
					//generate new middle plane
					calculateMiddlePlane(middlePlaneMap[pppp], *firstPPP, *secondPPP);
					planeParams = middlePlaneMap.find(pppp)->second;
					//					calculateMiddlePlane(params, firstPPP, secondPPP);
					//					middlePlaneMap.find(pppp)->second = params;
					if (middlePlaneIndicesMap.find(pppp) != middlePlaneIndicesMap.end()) {
						middlePlaneIndicesMap.find(pppp)->second.push_back(*point_it);
					} else {
						middlePlaneIndicesMap[pppp] = PointIndices(1, *point_it);
					}
				} else {
					//get "old" middle plane params
					planeParams = mpm_it->second;
					PointIndices pointIndices;
					getAllPointsFromNodes(pointIndices, NodeIndices(1, nodeIndex), octreeNodes, octree, pointCloud);
					if (middlePlaneIndicesMap.find(pppp) != middlePlaneIndicesMap.end()) {
						middlePlaneIndicesMap.find(pppp)->second.push_back(*point_it);
					} else {
						middlePlaneIndicesMap[pppp] = PointIndices(1, *point_it);
					}
				}
				//decide on which side of the middleplane the points mean lies

				if (planeParams.signedDistance(firstPPP->getPlaneCoG()) * planeParams.signedDistance(secondPPP->getPlaneCoG()) > 0) {
					//special T-Case (both cog on the same side of middleplane)
					//1-prefer "first" plane in pair - this may look random
					//					if(pppp.first->signedDistance(PlanePatch::getEigenVecFromPCL(mPointCloud.points[*point_it]))
					//							* pppp.first->signedDistance(pppp.second->getPlaneCoG()) > 0)
					//						finalPlanePointAssignmentMap[firstPPP].push_back(*point_it);
					//					else finalPlanePointAssignmentMap[pppp.first].push_back(*point_it);
					//2-prefer small distance
					finalPlanePointAssignmentMap[firstPPP].push_back(*point_it);
				} else {

					if (planeParams.signedDistance(firstPPP->getPlaneCoG()) < 0.f) {
						if (planeParams.signedDistance(pointCloud.points[*point_it].getVector3fMap()) < 0.f) {
							finalPlanePointAssignmentMap[firstPPP].push_back(*point_it);
						} else
							finalPlanePointAssignmentMap[secondPPP].push_back(*point_it);
					} else {
						if (planeParams.signedDistance(pointCloud.points[*point_it].getVector3fMap()) > 0.f) {
							finalPlanePointAssignmentMap[firstPPP].push_back(*point_it);
						} else
							finalPlanePointAssignmentMap[secondPPP].push_back(*point_it);
					}
				}
			}
		} else {
			if (node_idx_it->size() == 1) {//node has exactly one plane candidate, that it my lie in
				//assign node to plane
				finalPlaneNodeAssignmentMap[*(node_idx_it->begin())].push_back(octreeNodes[nodeIndex]);
			}
			//no plane candidate -> no assignment for this node!
		}
		nodeIndex++;
	}
	PlanePatches planes;
	for (MiddlePlaneMap::const_iterator mapit = middlePlaneMap.begin(); mapit != middlePlaneMap.end(); ++mapit) {
		const Plane3D& params = mapit->second;
		PointIndices pointIndices;
		MiddlePlaneIndicesMap::const_iterator mpimIt = middlePlaneIndicesMap.find(mapit->first);
		if (mpimIt != middlePlaneIndicesMap.end())
			pointIndices = mpimIt->second;
		if (pointIndices.size() > 3) {
			PlanePatchPtr plane(new PlanePatch(params.getPlaneNormal(), params.getPlaneDistanceToOrigin(), pointIndices, pointCloud,
					getSACDistanceFromDepth(octree.getMaxDepth(), octree)));
			planes.push_back(plane);
		}
	}
	if (mVis) mVis->publishPlaneMarker(planes, "middleplane");
}

/*****************************************************************************/

void StructureColoring::spreadNodes(const OcTree& octree, PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
		const PointCloud& pointCloud) {
	const NodePointers& octreeNodes = octree.getNodesOnDepth(octree.getMaxDepth());
	NodeIndexToPlanePatchPointers nodeToPlaneCandidates(octreeNodes.size());

	//determine candidate planes for each node (saved in nodePointersMap)
	//check against distance only
	PatchToNodePointersMap nodePointersMap;
	PatchToNodeIndicesMap nodeIndicesMap;
	//determineCandidatePlanes(nodeIndicesMap, nodePointersMap, octreeNodes);

	//refine node-to-plane multi-assignment through CC-filtering
	//save results in nodeToPlaneCandidates(n* -> p*)
	determineCandidatePlanes(nodeToPlaneCandidates, octreeNodes, extractedPlanes);

	//decide for each node, which planes are the best candidates and generate middleplane (if not already in map)
	//refine edges with middle plane sign check
	PatchToNodePointersMap finalPlaneNodeAssignmentMap;
	PatchToPointIndicesMap finalPlanePointAssignmentMap;
	refineEdges(finalPlaneNodeAssignmentMap, finalPlanePointAssignmentMap, nodeToPlaneCandidates, octreeNodes, octree, pointCloud);

	// reset mSegmented to false
	for (PlanePatchVector::iterator seg_iter = pointMapping.begin(); seg_iter != pointMapping.end(); ++seg_iter)
		seg_iter->reset();

	PlanePatches newPlanes;
	// add point-vectors to planes update mSegmented
	for (PatchToNodePointersMap::iterator it = finalPlaneNodeAssignmentMap.begin(); it != finalPlaneNodeAssignmentMap.end(); ++it) {
		const PlanePatch& pRef = *(it->first);
		PointIndices inliers;
		getAllPointsFromNodes(inliers, it->second, octree, pointCloud);
		PatchToPointIndicesMap::iterator pointIndicesMap_it = finalPlanePointAssignmentMap.find(it->first);
		if (pointIndicesMap_it != finalPlanePointAssignmentMap.end()) {
			inliers.reserve(inliers.size() + pointIndicesMap_it->second.size());
			for (PointIndices::const_iterator point_indices_it = pointIndicesMap_it->second.begin(); point_indices_it
					!= pointIndicesMap_it->second.end(); ++point_indices_it)
				inliers.push_back(*point_indices_it);
		}
		if (inliers.size() > mParams.mMinPointsInCC) {
			PlanePatchPtr ppp(new PlanePatch(inliers, pointCloud, pRef.getDistanceThreshold()));
			if (ppp) {
				ppp->newGrid(octree.getCellSizeFromDepth(octree.getMaxDepth()));
				ppp->grid()->blindPopulate(inliers, pointCloud);
				ppp->grid()->calculateStartPos();
				newPlanes.push_back(ppp);
			}
		}
	}
	extractedPlanes.clear();
	addPlanePatchesAndUpdateSegmented(extractedPlanes, pointMapping, newPlanes);
}

/*****************************************************************************/
/**
 * Iterate over all planes in extractedPlanes and merge planes that are similarly oriented
 * and each others center of gravity is on the other plane.
 * Connection via connected component is handled via "planesAreConnected"-check.
 * As std::merge is used in this method, Plane indices MUST be sorted! --> sort is being called on local copies .. //TODO Check if it hurts.
 */
void StructureColoring::mergePlanes(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, const PointCloud& pointCloud) {
	PlanePatches merged;
	CompArea compArea;
	extractedPlanes.sort(compArea);
	PlanePatches::iterator plane_it_it = extractedPlanes.begin();
	while (plane_it_it != extractedPlanes.end()) {
		float sacDist = (*plane_it_it)->getDistanceThreshold();
		std::vector<PlanePatches::iterator> mergeWithPlane;
		for (PlanePatches::iterator filteredPlane_it = extractedPlanes.begin(); filteredPlane_it
				!= extractedPlanes.end(); filteredPlane_it++) {
			if (filteredPlane_it != plane_it_it) {
				sacDist = std::max((*filteredPlane_it)->getDistanceThreshold(), sacDist);
				if (((*plane_it_it)->getPlane3D().checkNormal((*filteredPlane_it)->getPlane3D().getPlaneNormal(),
						mParams.mMergePlanesSimilarityFactor * mParams.mAngleEps))
						&& (*plane_it_it)->getPlane3D().checkDistance((*filteredPlane_it)->getPlaneCoG(), mParams.mMergePlanesSimilarityFactor * sacDist)
						&& (*filteredPlane_it)->getPlane3D().checkDistance((*plane_it_it)->getPlaneCoG(), mParams.mMergePlanesSimilarityFactor * sacDist)) {
					if (planesAreConnected(**plane_it_it, **filteredPlane_it, pointCloud)) {
						mergeWithPlane.push_back(filteredPlane_it);
					}
				}
			}
		}
		if (!mergeWithPlane.empty()) {
			pclPointIndices pinliers, outInliers;
			pinliers.indices = (*plane_it_it)->getInliers();
			std::sort(pinliers.indices.begin(), pinliers.indices.end());
			float cellSize = (*plane_it_it)->grid()->getCellSize();
			for (unsigned int i = 0; i < mergeWithPlane.size(); i++) {
				cellSize = std::min(cellSize, (*mergeWithPlane[i])->grid()->getCellSize());
				pclPointIndices pinliers2;
				pinliers2.indices = (*mergeWithPlane[i])->getInliers();
				std::sort(pinliers2.indices.begin(), pinliers2.indices.end());

				std::merge(pinliers.indices.begin(), pinliers.indices.end(), pinliers2.indices.begin(),
						pinliers2.indices.end(), std::back_inserter(outInliers.indices));
				pinliers.indices.clear();
				pinliers.indices.swap(outInliers.indices);
				extractedPlanes.erase(mergeWithPlane[i]);
			}
			for (PointIndices::const_iterator point_it = pinliers.indices.begin(); point_it != pinliers.indices.end(); point_it++) {
				pointMapping[*point_it].reset();
			}
			PlanePatchPtr pp;
			if (pinliers.indices.size() >= 3)
				pp.reset(new PlanePatch(pinliers.indices, pointCloud, sacDist));
			if (pp) {
				pp->newGrid(cellSize);
				pp->grid()->blindPopulate(pp->getInliers(), pointCloud);
				pp->grid()->calculateStartPos();
				merged.push_front(pp);
				plane_it_it = extractedPlanes.erase(plane_it_it);
			}
		} else
			++plane_it_it;
	}
	addPlanePatchesAndUpdateSegmented(extractedPlanes, pointMapping, merged);
}

/*****************************************************************************/

bool StructureColoring::planesAreConnected(const PlanePatch& pp1, const PlanePatch& pp2, const PointCloud& pointCloud) {
	const PointIndices& pp2Inliers = pp2.getInliers();
	for (PointIndices::const_iterator pp2Inliers_it = pp2Inliers.begin(); pp2Inliers_it != pp2Inliers.end(); ++pp2Inliers_it) {
		if (pp1.checkPointConnection(pointCloud.points[*pp2Inliers_it].getVector3fMap(),
				mParams.mConnectionNeighbors))
			return true;
	}
	return false;
}

/*****************************************************************************/

void StructureColoring::addPlanePatchesAndUpdateSegmented(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
		const PlanePatches& planePatches) {
	for (PlanePatches::const_iterator plane_it = planePatches.begin(); plane_it != planePatches.end(); ++plane_it) {
		extractedPlanes.push_front(*plane_it);
		const PointIndices& indices = (*plane_it)->getInliers();
		for (PointIndices::const_iterator it = indices.begin(); it != indices.end(); ++it) {
			if (pointMapping[*it])
				ROS_WARN("plane inliers are not disjunct anymore!");
			pointMapping[*it] = *plane_it;
		}
	}
}

/*****************************************************************************/

float StructureColoring::getSACDistanceFromDepth(const unsigned int& depth, const OcTree& octree) {
	float currentOctreeBinSize = octree.getCellSizeFromDepth(depth);
	float dist = std::max(mParams.mMinSACDistanceThreshold, std::min(mParams.mMaxSACDistanceThreshold, mParams.mSACOctreeFactor
			* currentOctreeBinSize));
	return dist;
}

/*****************************************************************************/

float StructureColoring::getHTDistanceFromDepth(const unsigned int& depth, const OcTree& octree) {
	float currentOctreeBinSize = octree.getCellSizeFromDepth(depth);
	float dist = std::min(mParams.mMaxHTDistanceThreshold, mParams.mHTOctreeBinSizeFactor * currentOctreeBinSize);
	return dist;
}

/*****************************************************************************/

void StructureColoring::getMinMaxRadiusFromDepth(float& minRadius, float& maxRadius, const OcTree& octree, const float& depth) {
	minRadius = mParams.mMinRadiusFactor * octree.getCellSizeFromDepth(depth);
	maxRadius = mParams.mMaxRadiusFactor * octree.getCellSizeFromDepth(depth);
}

/*****************************************************************************/

float StructureColoring::getAngleEpsFromDepth(const unsigned int& depth, const OcTree& octree) {
	float angleEps = mParams.mAngleEps * (1.f - (float) depth / (float) octree.getMaxDepth()) + mParams.mAngleEpsOnMinOctreeRes
			* ((float) depth / (float) octree.getMaxDepth());
	return angleEps;
}

/*****************************************************************************/

void StructureColoring::flipToViewport(Vec3& vec, const Vec3& viewport) {
	if (viewport.dot(vec) > 0) {
		vec *= -1.f;
	}
}

/*****************************************************************************/

double StructureColoring::segmentPlanesSAC(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, const PointCloudPtr& pointCloud)
{
    ROS_INFO("segmentPlanesSAC");
    ros::Time start = ros::Time::now();

    mLastCellSizeWithNormals = mParams.mMinSACDistanceThreshold;
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    size_t nrPoints = pointCloud->points.size();
    indices->indices.reserve(nrPoints);
    for (size_t i = 0; i < nrPoints; ++i)
    {
        indices->indices.push_back(i);
    }
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
	seg.setMaxIterations(mParams.mPclSACmaxIter);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (mParams.mMinSACDistanceThreshold);
	seg.setInputCloud(pointCloud);

    ros::Time end = ros::Time::now();
    ros::Duration dt = end - start;
    ROS_INFO("preprocessing time: %f\n", dt.toSec());

    unsigned int failcounter = 0;
    unsigned int maxIter = 100;

    while (indices->indices.size () > mParams.mMinPointsInCC && failcounter < maxIter)
    {
        ros::Time start2 = ros::Time::now();
        seg.setIndices(indices);
        seg.segment (*inliers, coefficients);
        end = ros::Time::now();
        dt = end - start2;
        if (mParams.mVerbose)
            ROS_INFO("segmentation time: %f", dt.toSec());

        if (inliers->indices.size() >= mParams.mMinPointsInCC)
        {

            Vec3 norm(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
            // RANSAC model Coefficients: ax + by + cz + d = 0
            // my model: n*p - d = 0 with d > 0
            // thus i must switch the distance's sign!
            float distance = -1.f * coefficients.values[3];
            if (distance < 0) {
                distance *= -1.f;
                norm *= -1.f;
            }
            PlanePatchPtr pp = boost::make_shared<PlanePatch>(norm, distance, inliers->indices, *pointCloud, mParams.mMinSACDistanceThreshold);

            // segment plane into connected components
            // fake octree nodes for the points..
			std::vector<NodePointers> CCInliers;
			OcTreePtr octreePtr(new OcTree(mParams.mMinOctreeResolution, mParams.mRho_max, mParams.mPrincipalVarFactor,
					mParams.mMinPointsForNormal, mParams.mCurvThreshold, mParams.mCurv2Threshold, mParams.mSqDistFactor, 5));
			NodePointers inlierOctreeNodes;
			for (PointIndices::const_iterator in_it = pp->getInliers().begin();
					in_it != pp->getInliers().end(); ++in_it) {
				NodePtr n = new spatialaggregate::OcTreeNode<float, ValueClass>();
//
//				float minpx = pointCloud->points[*in_it].x - mMinSACDistanceThreshold;
//				float minpy = pointCloud->points[*in_it].y - mMinSACDistanceThreshold;
//				float minpz = pointCloud->points[*in_it].z - mMinSACDistanceThreshold;
//				OcTree::OctreeKey minpos(minpx, minpy, minpz, octreePtr->getOctreePtr());
//
//				float maxpx = pointCloud->points[*in_it].x + mMinSACDistanceThreshold;
//				float maxpy = pointCloud->points[*in_it].y + mMinSACDistanceThreshold;
//				float maxpz = pointCloud->points[*in_it].z + mMinSACDistanceThreshold;
//				OcTree::OctreeKey maxpos(maxpx, maxpy, maxpz, octreePtr->getOctreePtr());
//
//				float px = pointCloud->points[*in_it].x;
//				float py = pointCloud->points[*in_it].y;
//				float pz = pointCloud->points[*in_it].z;
//				OcTree::OctreeKey pos(px, py, pz, octreePtr->getOctreePtr());

				n->depth_ = 0;
				n->value_.meanPos(0) = pointCloud->points[*in_it].x;
				n->value_.meanPos(1) = pointCloud->points[*in_it].y;
				n->value_.meanPos(2) = pointCloud->points[*in_it].z;

				n->value_.normal = norm;
				n->value_.stable = true;

				// store index in numPoints....
				n->value_.numPoints = *in_it;

				inlierOctreeNodes.push_back(n);
			}

			filterConnectedNodes(CCInliers, pp->getPlane3D(), inlierOctreeNodes, mParams.mMinSACDistanceThreshold);

			bool ccextracted = false;
			ROS_INFO("found %zu connected components", CCInliers.size());
			for (unsigned int cc = 0; cc < CCInliers.size(); cc++) {

				PointIndices CCIndices;

				for( unsigned int ccn = 0; ccn < CCInliers[cc].size(); ccn++ )
					CCIndices.push_back( CCInliers[cc][ccn]->value_.numPoints );

				size_t sizeOfInliers = CCIndices.size();
				if (sizeOfInliers >= mParams.mMinPointsInCC) {
//					ROS_INFO("connected component has enough points :)");
					ccextracted = true;
					PlanePatchPtr ccpp;
					ccpp.reset(new PlanePatch(pp->getPlane3D().getPlaneNormal(), pp->getPlane3D().getPlaneDistanceToOrigin(), CCIndices, *pointCloud, pp->getDistanceThreshold()));

					extractedPlanes.push_back(ccpp);
					for (PointIndices::const_iterator in_it = CCIndices.begin();
							in_it != CCIndices.end(); ++in_it)
					{
						pointMapping[*in_it] = ccpp;
					}

			        // remove indices of plane inliers from the indices used for the next iteration:
			        start2 = ros::Time::now();
			        PointIndices current = indices->indices;
			        std::sort(CCIndices.begin(), CCIndices.end());
			        indices->indices.clear();
			        std::set_difference(current.begin(), current.end(), CCIndices.begin(), CCIndices.end(), std::back_inserter(indices->indices));
			        end = ros::Time::now();
			        dt = end - start2;
			        if (mParams.mVerbose)
			            ROS_INFO("extraction time: %f, %zu left", dt.toSec(), indices->indices.size());

				} else {
//					ROS_INFO("connected component did not have enough points :(");
				}

			}
			if (!ccextracted)
				failcounter++;

			for( unsigned int i = 0; i < inlierOctreeNodes.size(); i++ )
				delete inlierOctreeNodes[i];

        }
        else
        {
            ROS_ERROR("Could not estimate a planar model for the given dataset.\n");
            break;
        }

    }
    dt = end - start;
    ROS_INFO("segmentPlanesSAC time: %f", dt.toSec());
    return dt.toSec();
}

double StructureColoring::segmentCylindersSAC(CylinderPatches& cylinderPatches, CylinderPatchVector& pointMapping, OcTree& octree, const PointCloudPtr& pointCloud)
{
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;

    ROS_INFO("segmentCylinderSAC");
    double start = pcl::getTime();

	pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    size_t nrPoints = pointCloud->points.size();
    indices->indices.reserve(nrPoints);
    for (size_t i = 0; i < nrPoints; ++i)
    {
        indices->indices.push_back(i);
    }

    NormalCloud::Ptr normalCloud(new NormalCloud);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (pointCloud);
	ne.setSearchMethod (tree);
	// Use all neighbors in a sphere of radius ..
	ne.setRadiusSearch (mParams.mMinOctreeResolution); // im normalen Verfahren wird die CellSize durch 2 genommen, um auf den Radius zu kommen, das könnte hier zu klein sein
	//ne.setKSearch (50);
	ne.setViewPoint(0.f, 0.f, 0.f);
	// Compute the features
	ne.setInputCloud (pointCloud);
	ne.compute (*normalCloud);
    ROS_INFO("segmentCylinderSAC: normals computed.");

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(mParams.mPclSACmaxIter);

	//seg.setDistanceThreshold(sacDist);
	seg.setDistanceThreshold(mParams.mMinSACDistanceThreshold);
	seg.setNormalDistanceWeight(mParams.mNormalDistanceWeight);
	seg.setRadiusLimits (mParams.mMinCylinderRadius, mParams.mMaxCylinderRadius);

	seg.setInputCloud(pointCloud);
	seg.setInputNormals(normalCloud);

    unsigned int failcounter = 0;
    while ((indices->indices.size () > (nrPoints/2)) && (failcounter < mParams.mTriesOnEachDepth))
    {
        ROS_INFO("segmentCylinderSAC: segmenting, %zu points left ...", indices->indices.size());
        seg.setIndices(indices);
    	seg.segment(*inliers, coefficients);

        if (inliers->indices.size() >= mParams.mMinPointsInCC)
        {
            Vec3 pointOnAxis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
            Vec3 axisDirection(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
            float radius(coefficients.values[6]);
            CylinderPatchPtr cylinder = boost::make_shared<CylinderPatch>(inliers->indices, *pointCloud, pointOnAxis, axisDirection, radius, mParams.mMinSACDistanceThreshold);
            if (checkCylinder(cylinder, pointCloud))
            {
                cylinderPatches.push_back(cylinder);
                for (PointIndices::const_iterator in_it = cylinder->getInliers().begin();
                        in_it != cylinder->getInliers().end(); ++in_it)
                {
                    pointMapping[*in_it] = cylinder;
                }
            }
            else
                failcounter++;
        }
        else {
            ROS_ERROR("RANSAC: no cylinder found, aborting");
            break;
        }
        // remove indices of plane inliers from the indices used for the next iteration:
        PointIndices current = indices->indices;
        indices->indices.clear();
        std::set_difference(current.begin(), current.end(), inliers->indices.begin(), inliers->indices.end(), std::back_inserter(indices->indices));
    }
    double end = pcl::getTime();
    double elapsedTime = end - start;
    ROS_INFO("segmentCylinderSAC time: %f", elapsedTime);
    return elapsedTime;
}

bool StructureColoring::checkCylinderDimensions(const CylinderPatchPtr& cylinder) const
{
    return ((cylinder->getRadius() > mParams.mMinCylinderRadius) && (cylinder->getRadius() < mParams.mMaxCylinderRadius));
}

bool StructureColoring::checkCylinder(const CylinderPatchPtr& cylinder, const PointCloudPtr& pointCloud) const
{
    float cellSize = mParams.mMinOctreeResolution;
    CylinderGridMap grid(cellSize, cylinder, *pointCloud);
    size_t nrCells = grid.getCells().size();
    assert(nrCells == (grid.getWidth()*grid.getHeight()));
            
    int count = 0;
    for (size_t i = 0; i < nrCells; ++i)
        if (grid.getCells()[i]) ++count;
    float occupied = (float)count/(float)nrCells;
    return occupied > mParams.mOccupiedRatio;
}

