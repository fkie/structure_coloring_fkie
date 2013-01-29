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

#ifndef _PlanePatch_h_
#define _PlanePatch_h_

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <structureColoring/grids/GridMap.h>
#include <structureColoring/structures/Plane3D.h>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/StdVector>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <list>

class PlanePatch{
public:
	typedef boost::shared_ptr<PlanePatch> PlanePatchPtr;
	typedef boost::shared_ptr<const PlanePatch> PlanePatchConstPtr;
	typedef std::list<PlanePatchPtr> PlanePatches;

	typedef std::vector<int> PointIndices;
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointXYZRGB ColoredPointT;
	typedef pcl::PointXYZI IntensityPointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointCloud<ColoredPointT> ColoredPointCloud;
	typedef pcl::PointCloud<IntensityPointT> IntensityPointCloud;
	typedef std::pair<unsigned int, unsigned int> Pair2ui;
	typedef Eigen::Vector3f Vec3;
	typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Points;
	typedef boost::shared_ptr<GridMap> GridMapPtr;
	typedef boost::shared_ptr<const GridMap> GridMapConstPtr;

	template<typename PointCloudType>
	PlanePatch(const Eigen::Vector3f& normal, const float distance, const PointIndices& inliers,
			const PointCloudType& pointCloud, const float distanceThreshold = -1.f);

	template<typename PointCloudType>
	PlanePatch(const PointIndices& inliers, const PointCloudType& pointCloud, const float distanceThreshold);

	PlanePatch(const Vec3& normal, const float distance, const float distanceThreshold, const Points& brVertices,
			const float texWidthRatio, const float texHeightRatio, const cv::Mat& heightMap, const cv::Mat& textureMap);

	PlanePatch();

//	virtual ~PlanePatch();

//setter:
	template<typename PointCloudType>
	void addPoints(const PointIndices& pointIndices, const PointCloudType& pointCloud);

	float& xMin() { return mXMin; }
	float& yMin() { return mYMin; }
	float& xMax() { return mXMax; }
	float& yMax() { return mYMax; }

	GridMapPtr grid() { return mGrid; }
	void setGrid(GridMap* newGridMapPtr){ mGrid.reset(newGridMapPtr); }

	void setId(size_t id) { mPlaneId = id; }

//getter:
	const Points& getBRVertices() const { return mBRVertices; }
	const Points& getBBVertices() const { return mBBVertices;}

	const cv::Mat& getTextureMap() const {
		assert(mTextureMap.cols != 0 && mTextureMap.rows != 0);
		return mTextureMap;
	}
	const cv::Mat& getHeightMap() const {
		assert(mHeightMap.cols != 0 && mHeightMap.rows != 0);
		return mHeightMap;
	}
	const cv::Mat& getNormalMap() const {
		assert(mNormalMap.cols != 0 && mNormalMap.rows != 0);
		return mNormalMap;
	}
	float getTextureWidthRatio() const { return mTextureWidthRatio; }
	float getTextureHeightRatio() const { return mTextureHeightRatio; }

	const Vec3& getPlaneCoG() const { return mCoG; }
	const Plane3D& getPlane3D() const { return mPlane; }

	void getLimitsXY(float& xMin, float& xMax, float& yMin, float& yMax) const;
	const float& xMin() const { return mXMin; }
	const float& yMin() const { return mYMin; }
	const float& xMax() const { return mXMax; }
	const float& yMax() const { return mYMax; }
	float getArea() const { return (mXMax - mXMin) * (mYMax - mYMin); }
	const PointIndices& getInliers() const { return mInlierIndices; }
	float getDistanceThreshold() const { return mDistanceThreshold; }

	GridMapConstPtr grid() const { return mGrid; }
	bool hasGrid() const{ return mGrid; }

	size_t getId() const { return mPlaneId; }


	//helper functions
	bool checkPointConnection(const Vec3& p, const int connectionNeighbors) const;

	float distanceToOBB(const Vec3& p) const;

	void newGrid(const float cellSize);

	void computeOrientedBoundingBox();
	void computeBoundingRectangle();

	//compute texture etc.
	void computeTextureSize(float& texPixSize, unsigned int& wPot, unsigned int& hPot, const float texelSizeFactor);
	static void compute2PotTexSize(float& widthRatio, float& heightRatio, unsigned int& wPot, unsigned int& hPot,
			const unsigned int width, const unsigned int height);
	void computeRGBTextureMap(const ColoredPointCloud& pointCloud, const float texelSizeFactor = 2.f,
			const unsigned int dilateIterations = 0);
	void computeIntensityTextureMap(const IntensityPointCloud& pointCloud, const float texelSizeFactor = 2.f,
			const unsigned int dilateIterations = 0);
	void computeAlphaMap(const PointCloud& pointCloud, const float texelSizeFactor = 2.f,
			const unsigned int dilateIterations = 0);

	template<typename PointCloudType>
	void computeHeightMap(const float texturePixelSize, const PointCloudType& pointCloud,
			const unsigned int dilateIterations);

	void computeNormalMap();

	// private copy constructor: (we do not copy PlanePatches)
private:
	PlanePatch(const PlanePatch&) :
		mPlane(){
		throw std::runtime_error("Copy Constructor not implemented");
	}
private:
	PlanePatch& operator=(const PlanePatch&) {
		throw std::runtime_error("operator= not implemented");
	}

private:
	template<typename PointCloudType>
	void initializeTransformationsWithoutPCA(const PointCloudType& pointCloud);

	template<typename PointCloudType>
	void initializeTransformationsWithPCA(const PointCloudType& pointCloud);

	template<typename PointCloudType>
	void initializeWithoutPCA(const PointCloudType& pointcloud);

	template<typename PointCloudType>
	void initializeWithPCA(const PointCloudType& pointCloud);

	template<typename PointCloudType>
	void calculateMinMax(const PointCloudType& pointCloud);


	template<typename PointCloudType>
	void rotateToPrincipalAxis(Eigen::Matrix3f& m, const PointCloudType& pointCloud, const Vec3& greatestEigenVector =
			Vec3::Zero()) const;

	//member variables
	Plane3D mPlane;
	Vec3 mCoG;
	float mXMin, mXMax, mYMin, mYMax, mZMin, mZMax; // dimensions in plane coordinate system
	PointIndices mInlierIndices;
	Points mBRVertices;
	Points mBBVertices;
	GridMapPtr mGrid;
	cv::Mat mTextureMap;
	cv::Mat mHeightMap;
	cv::Mat mNormalMap;
	float mTextureWidthRatio, mTextureHeightRatio;
	float mDistanceThreshold;
	size_t mPlaneId;
};

/*****************************************************************************/

class CompArea
{
public:
	bool operator()(PlanePatch::PlanePatchConstPtr pp1, PlanePatch::PlanePatchConstPtr pp2){
		return pp2->getArea() < pp1->getArea();
	}
};

/*****************************************************************************/

class CompPlanePtr
{
public:
	bool operator()(const PlanePatch::PlanePatchConstPtr pp1, const PlanePatch::PlanePatchConstPtr pp2) const {
		return pp1.get() < pp2.get();
	}
};

/*****************************************************************************/

class CompPlanePtrPair
{
	typedef std::pair<PlanePatch::PlanePatchConstPtr, PlanePatch::PlanePatchConstPtr> PlaneConstPtrPair;
public:
	bool operator()(const PlaneConstPtrPair ppp1, const PlaneConstPtrPair ppp2) const {
		if (ppp1.first.get() != ppp2.first.get()) return ppp1.first.get() < ppp2.first.get();
		return ppp1.second.get() < ppp2.second.get();
	}
};

/*****************************************************************************/

template<typename PointCloudType>
PlanePatch::PlanePatch(const Eigen::Vector3f& normal, const float distance, const PointIndices& inliers,
		const PointCloudType& pointCloud, const float distanceThreshold)
	: mPlane(normal, distance), mInlierIndices(inliers),
	  mDistanceThreshold(distanceThreshold) {
	assert(inliers.size()>= 3);
	initializeWithoutPCA(pointCloud);
}

/*****************************************************************************/

template<typename PointCloudType>
PlanePatch::PlanePatch(const PointIndices& inliers, const PointCloudType& pointCloud, const float distanceThreshold)
	: mInlierIndices(inliers),
	  mDistanceThreshold(distanceThreshold) {
	assert(inliers.size()>= 3);
	initializeWithPCA(pointCloud);
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::addPoints(const PointIndices& pointIndices, const PointCloudType& pointCloud) {
	mInlierIndices.reserve(mInlierIndices.size() + pointIndices.size());
	for (PointIndices::const_iterator pit = pointIndices.begin(); pit != pointIndices.end(); ++pit) {
		mInlierIndices.push_back(*pit);
		Vec3 tp = mPlane.transformToXYPlane(pointCloud.points[*pit].getVector3fMap());
		if (tp.x() < mXMin) mXMin = tp.x();
		if (tp.x() > mXMax) mXMax = tp.x();
		if (tp.y() < mYMin) mYMin = tp.y();
		if (tp.y() > mYMax) mYMax = tp.y();
	}
	computeOrientedBoundingBox();
	computeBoundingRectangle();
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::computeHeightMap(const float texturePixelSize, const PointCloudType& pointCloud,
		const unsigned int dilateIterations) {
	unsigned int hPot = mTextureMap.rows;
	unsigned int wPot = mTextureMap.cols;
	mHeightMap = cv::Mat::zeros(hPot, wPot, CV_32FC1);
	cv::Mat texCounter(cv::Mat::zeros(hPot, wPot, CV_8UC1));
	float distThresh = getDistanceThreshold();
	for (PointIndices::const_iterator point_it = mInlierIndices.begin(); point_it != mInlierIndices.end(); ++point_it) {
		float distanceToPlane = mPlane.signedDistance(pointCloud.points[*point_it].getVector3fMap());
		float distToMaxDistRatio = distanceToPlane / distThresh;
		float normalizedRatio = 0.5 * distToMaxDistRatio;
		Vec3 p = mPlane.transformToXYPlane(pointCloud.points[*point_it].getVector3fMap());
		unsigned int x = fabs(p[0] - mXMin) / texturePixelSize;
		unsigned int y = fabs(p[1] - mYMin) / texturePixelSize;
		mHeightMap.at<float> (y, x) += normalizedRatio + 0.5f;
		texCounter.at<char> (y, x) += 1;
	}
	for (unsigned int hind = 0; hind < hPot; ++hind) {
		for (unsigned int wind = 0; wind < wPot; ++wind) {
			char count = texCounter.at<char> (hind, wind);
			if (count != 0) {
				mHeightMap.at<float> (hind, wind) /= count;
			}
		}
	}

	if (dilateIterations)
		cv::dilate(mHeightMap, mHeightMap, cv::Mat(), cv::Point(-1, -1), dilateIterations);
	cv::GaussianBlur(mHeightMap, mHeightMap, cv::Size(3, 3), 2, 2);
	computeNormalMap();
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::initializeTransformationsWithoutPCA(const PointCloudType& pointCloud) {
	mCoG = Eigen::Vector3f(0, 0, 0);
	for (PointIndices::const_iterator it = mInlierIndices.begin(); it != mInlierIndices.end(); ++it) {
		mCoG += pointCloud.points[*it].getVector3fMap();
	}
	mCoG /= (float) mInlierIndices.size();
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::initializeTransformationsWithPCA(const PointCloudType& pointCloud) {
	Points points;
	mCoG = Eigen::Vector3f(0, 0, 0);
	for (PointIndices::const_iterator it = mInlierIndices.begin(); it != mInlierIndices.end(); ++it) {
		const Vec3& p = pointCloud.points[*it].getVector3fMap();
		points.push_back(p);
		mCoG += p;
	}
	mCoG /= (float) mInlierIndices.size();

	Eigen::Matrix3f cov(Eigen::Matrix3f::Zero());
	for (Points::const_iterator it = points.begin(); it != points.end(); ++it) {
		Vec3 p = *it - mCoG;
		cov += p * p.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
	es.compute(cov);
	const Eigen::Matrix<float, 3, 3>& eigen_vectors = es.eigenvectors();

	mPlane = Plane3D(Vec3(eigen_vectors.col(0)), eigen_vectors.col(0).dot(mCoG));
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::initializeWithoutPCA(const PointCloudType& pointCloud) {
	initializeTransformationsWithoutPCA(pointCloud);
	calculateMinMax(pointCloud);
	computeOrientedBoundingBox();
	computeBoundingRectangle();
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::initializeWithPCA(const PointCloudType& pointCloud) {
	initializeTransformationsWithPCA(pointCloud);
	calculateMinMax(pointCloud);
	computeOrientedBoundingBox();
	computeBoundingRectangle();
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::calculateMinMax(const PointCloudType& pointCloud) {
	mXMin = mYMin = mZMin = std::numeric_limits<float>::max();
	mXMax = mYMax = mZMax = -std::numeric_limits<float>::max();

	//transform from "normal" xyz-coordinates to plane-coordinate system
	//compute min/max of BB/BR in three dimensions
	for (PointIndices::const_iterator it = mInlierIndices.begin(); it != mInlierIndices.end(); ++it) {
		const Eigen::Vector3f& p = mPlane.transformToXYPlane(pointCloud.points[*it].getVector3fMap());
		if (p(0) < mXMin) {
			mXMin = p(0);
		}
		if (p(0) > mXMax) {
			mXMax = p(0);
		}
		if (p(1) < mYMin) {
			mYMin = p(1);
		}
		if (p(1) > mYMax) {
			mYMax = p(1);
		}
		if (p(2) < mZMin)
			mZMin = p(2);
		if (p(2) > mZMax)
			mZMax = p(2);
	}
}

/*****************************************************************************/

template<typename PointCloudType>
void PlanePatch::rotateToPrincipalAxis(Eigen::Matrix3f& m, const PointCloudType& pointCloud, const Vec3& greatestEigenVector) const {
	Vec3 v(Vec3::Zero());
	if (greatestEigenVector == v) {
		// use PCA to rotate the coordinate system of the plane, s.t. e1 coincides with the first principal axis
		Eigen::Matrix3f cov(Eigen::Matrix3f::Zero());
		for (PointIndices::const_iterator it = mInlierIndices.begin(); it != mInlierIndices.end(); ++it) {
			Eigen::Vector3f p = getEigenVecFromPCL(pointCloud.points[*it]) - mCoG;
			cov += p * p.transpose();
		}
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
		es.compute(cov);
		v = es.eigenvectors().col(2);

	} else
		v = greatestEigenVector;
	Vec3 e1(1, 0, 0);
	e1 = m * e1;
	float rotAngle = acos(e1.dot(v));
	m = m * Eigen::AngleAxisf(-rotAngle, mPlane.getPlaneNormal());
}

#endif /*_PlanePatch_h_*/
