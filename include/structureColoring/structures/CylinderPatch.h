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

#ifndef _CylinderPatch_h_
#define _CylinderPatch_h_

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <vector>
#include <limits.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <list>
#include <structureColoring/structures/Cylinder3D.h>

class CylinderPatch : public Cylinder3D{
public:
	typedef boost::shared_ptr<CylinderPatch> CylinderPatchPtr;
	typedef boost::shared_ptr<const CylinderPatch> CylinderPatchConstPtr;
	typedef std::list<CylinderPatchPtr> CylinderPatches;

	typedef std::vector<int> PointIndices;
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointXYZRGB ColoredPointT;
	typedef pcl::PointXYZI IntensityPointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointCloud<ColoredPointT> ColoredPointCloud;
	typedef pcl::PointCloud<IntensityPointT> IntensityPointCloud;
	typedef std::pair<unsigned int, unsigned int> Pair2ui;
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Matrix3f Mat3;
	typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Points;


	CylinderPatch() :
		Cylinder3D(), mCylinderId(0), mTextureMap(NULL), mHeightMap(NULL),
				mNormalMap(NULL) {
	}

	template<typename PointCloudType>
	CylinderPatch(const PointIndices& inliers, const PointCloudType& pointCloud, const Vec3& pointOnAxis,
			const Vec3& axisDirection, const float radius, const float distThreshold) :
		Cylinder3D(pointOnAxis, axisDirection, radius), mDistThreshold(distThreshold), mPointIndices(inliers),
		mCylinderId(0), mTextureMap(NULL), mHeightMap(NULL), mNormalMap(NULL) {
		computeTransformation();
		initialize(inliers, pointCloud);
	}

	template<typename PointCloudType>
	CylinderPatch(const PointIndices& inliers, const PointCloudType& pointCloud, const Vec3& pointOnAxis,
			const Vec3& axisDirection, const float radius, const Vec3& rotationAxis, const float rotationAngle, const float distThreshold) :
		Cylinder3D(pointOnAxis, axisDirection, radius, rotationAxis, rotationAngle),
		mDistThreshold(distThreshold), mPointIndices(inliers), mCylinderId(0), mTextureMap(NULL), mHeightMap(NULL), mNormalMap(NULL) {
		setTransformation(rotationAngle, rotationAxis);
		initialize(inliers, pointCloud);
	}

	virtual ~CylinderPatch(){
		if (mTextureMap) delete mTextureMap;
		if (mHeightMap) delete mHeightMap;
		if (mNormalMap) delete mNormalMap;
	}

//	template<typename PointType>
//	static Vec3 getEigenVecFromPCL(const PointType& point) {
//		return point.getVector3fMap();
//	}

//	void setAxis(const Vec3& axis ){mAxisDirection = axis;}
//	void setRadius(float rad){mRadius = rad;}
//	void setMidPoint (const Vec3& midPoint){mMidPoint = midPoint;}
//	void setPointOnAxis(const Vec3& poa){mPointOnAxis = poa;}
	void setId(const size_t& id){mCylinderId = id;}

//getter
	const Points& getBBVertices() const {return mBBVertices;}
//	const Vec3& getPointOnAxis() const {return mPointOnAxis;}
//	const Vec3& getAxisDirection() const {return mAxisDirection;}
//	const float& getRadius() const {return mRadius;}
	float getAxisMin() const {return mAxisMin;}
	float getAxisMax() const {return mAxisMax;}
	void getAxisLimits(float& min,float& max) const {min = mAxisMin; max = mAxisMax;}
	const float& getHeight()const{return mHeight;}
	const Vec3& getTopPoint()const{return mTopPoint;}
	const Vec3& getMidPoint()const{return mMidPoint;}
//	const Vec3& getRotationAxis()const{return mRotAxis;}
//	const float& getRotationAngle()const{return mRotAngle;}
//	const Mat3& getTransformRot()const{return mTransformRot;}
//	const Mat3& getReTransformRot()const{return mReTransformRot;}
	const PointIndices& getInliers()const{return mPointIndices;}
	const size_t& getId()const{return mCylinderId;}

	const cv::Mat* getTextureMap() const {
		return mTextureMap;
	}
	const cv::Mat* getHeightMap() const {
		return mHeightMap;
	}
	const cv::Mat* getNormalMap() const {
		return mNormalMap;
	}

// inlier checks
	bool checkDistance(const Vec3& point) const{
		return Cylinder3D::checkDistance(point, mDistThreshold);
	}

//	bool checkNormal(const Vec3& normal, const Vec3& point, const float& mAngleEps) const{
//		Vec3 transformedPoint;
//		transformToCylinder(transformedPoint, point);
//		Eigen::Vector2f dirFromCenter(transformedPoint.x(), transformedPoint.y());
//		dirFromCenter.normalize();
//
//		Vec3 normalCylinder = mTransformRot * normal;
//		Eigen::Vector2f normal2 = normalCylinder.block<2,1>(0,0);
//
//		return fabsf(dirFromCenter.dot(normal2)) >= cosf(mAngleEps);
//	}

	bool checkHeight(const Vec3& point, const float& heightDev){
		Vec3 transformedPoint;
		transformToCylinder(transformedPoint, point);
		if( transformedPoint.z() < mAxisMax + heightDev * mHeight && transformedPoint.z() > mAxisMin - heightDev * mHeight) return true;
		return false;
	}

	template<typename PointCloudType>
	void addPoints(const PointIndices& pointIndices, const PointCloudType& pointCloud){
		mPointIndices.reserve(mPointIndices.size() + pointIndices.size());
		for(PointIndices::const_iterator pi_it = pointIndices.begin();pi_it != pointIndices.end(); ++pi_it){
			mPointIndices.push_back(*pi_it);
		}
        this->addToHeight(pointIndices, pointCloud);

	}

//	/**
//	 * \brief Compute distance from point to cylinder surface.
//	 * \param point Compute this point's distance.
//	 */
//	float distanceToCylinderSurface(const Vec3& point) const{
//		return distanceToAxis(point) - mRadius;
//	}
//
//	float distanceToAxis(const Vec3& point) const;

//	void projectPoint(Vec3& outPoint, const Vec3& point) const;
//	void transformToCylinder(Vec3& outPoint, const Vec3& point) const { outPoint = mTransformRot * (point + mTransformTrans);}
//	void transformToXYZCoords(Vec3& outPoint, const Vec3& point) const { outPoint = (mReTransformRot * point) - mTransformTrans;}

//	void computeTexture(float texturePixelSize);
private:
//	/**
//	 * \brief Calculate transformations from and to cylinder coordinate system.
//	 */
//	void computeTransformation();

	/**
	 * \brief Set transformations from and to cylinder coordinate system.
	 */
	void setTransformation(const float rotationAngle, const Vec3& rotationAxis);

	template<typename PointCloudType>
	void initialize(const PointIndices& pointIndices, const PointCloudType& pointCloud);

	template<typename PointCloudType>
	void addToHeight(const PointIndices& pointIndices, const PointCloudType& pointCloud);

	void computeOrientedBoundingBox();

//	Vec3 mPointOnAxis;
//	Vec3 mAxisDirection;
//	float mRadius;

//	Vec3 mRotAxis;
//	float mRotAngle;
//	Mat3 mTransformRot, mReTransformRot;
//	Vec3 mTransformTrans;
	float mAxisMin, mAxisMax;
	float mHeight;
	Vec3 mTopPoint;
	Vec3 mMidPoint;

	float mDistThreshold;

	PointIndices mPointIndices;
	Points mBBVertices;
	size_t mCylinderId;

	cv::Mat* mTextureMap;
	cv::Mat* mHeightMap;
	cv::Mat* mNormalMap;
};

//inline float CylinderPatch::distanceToAxis(const Vec3& point) const {
//	Vec3 transformedPoint;
//	transformToCylinder(transformedPoint, point);
//	Eigen::Vector2f pointOnHeightZ(transformedPoint.x(), transformedPoint.y());
//	return pointOnHeightZ.norm();
//}

//inline void CylinderPatch::projectPoint(Vec3& outPoint, const Vec3& point) const{
//	Vec3 transformedPoint;
//	transformToCylinder(transformedPoint, point);
//	Vec3 pOnAxisSameZ(0.f, 0.f, transformedPoint.z());
//	Vec3 connection = transformedPoint - pOnAxisSameZ;
//	float connectionLength = connection.norm();
//	outPoint = pOnAxisSameZ + connection * (mRadius / connectionLength);
//}

template<typename PointCloudType>
void CylinderPatch::initialize(const PointIndices& pointIndices, const PointCloudType& pointCloud){
	//calculate mAxisMin/mAxisMax (on z axis) for bounding box
	mAxisMin = std::numeric_limits<float>::max();
	mAxisMax = -std::numeric_limits<float>::max();
	for(PointIndices::const_iterator pit = pointIndices.begin(); pit != pointIndices.end(); ++pit){
		Vec3 p;
		transformToCylinder(p, pointCloud.points[*pit].getVector3fMap());

		if (p.z() < mAxisMin) mAxisMin = p.z();
		if (p.z() > mAxisMax) mAxisMax = p.z();
	}
	mHeight = fabs(mAxisMax - mAxisMin);
	Vec3 tp(0.f, 0.f, mAxisMax);
	Vec3 mp(0.f, 0.f, (mAxisMax + mAxisMin) * 0.5f);
	transformToXYZCoords(mMidPoint, mp);
	transformToXYZCoords(mTopPoint, tp);

	computeOrientedBoundingBox();
}

template<typename PointCloudType>
void CylinderPatch::addToHeight(const PointIndices& pointIndices, const PointCloudType& pointCloud){
	for(PointIndices::const_iterator pit = pointIndices.begin(); pit != pointIndices.end(); ++pit){
		Vec3 p;
		transformToCylinder(p, pointCloud.points[*pit].getVector3fMap());

		if (p.z() < mAxisMin){
			mAxisMin = p.z();
		}
		if (p.z() > mAxisMax){
			mAxisMax = p.z();
		}
	}
	mHeight = fabs(mAxisMax - mAxisMin);
	Vec3 tp(0.f, 0.f, mAxisMax);
	Vec3 mp(0.f, 0.f, (mAxisMax + mAxisMin) * 0.5f);
	transformToXYZCoords(mMidPoint, mp);
	transformToXYZCoords(mTopPoint, tp);

	computeOrientedBoundingBox();
}

#endif /*_CylinderPatch_h_*/
