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

#ifndef CYLINDER3D_H_
#define CYLINDER3D_H_

#include <Eigen/Dense>
#include <structureColoring/RotationHelper.h>
#include <boost/shared_ptr.hpp>

class Cylinder3D {
public:
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Matrix3f Mat3;
	typedef boost::shared_ptr<Cylinder3D> Cylinder3DPtr;

	Cylinder3D(): mPointOnAxis(0.f, 0.f, 0.f), mAxisDirection(0.f, 0.f, 0.f), mRadius(0){}
	Cylinder3D(Vec3 pointOnAxis, Vec3 axisDirection, float radius)
	: mPointOnAxis(pointOnAxis), mAxisDirection(axisDirection), mRadius(radius) {}
	Cylinder3D(Vec3 pointOnAxis, Vec3 axisDirection, float radius, const Vec3& rotationAxis, const float rotationAngle)
	: mPointOnAxis(pointOnAxis), mAxisDirection(axisDirection), mRadius(radius) {setTransformation(rotationAngle, rotationAxis);}

//getter
	const Vec3& getPointOnAxis() const {return mPointOnAxis;}
	const Vec3& getAxisDirection() const {return mAxisDirection;}
	const float& getRadius() const {return mRadius;}
	const Vec3& getRotationAxis()const{return mRotAxis;}
	const float& getRotationAngle()const{return mRotAngle;}
	const Mat3& getTransformRot()const{return mTransformRot;}
	const Mat3& getReTransformRot()const{return mReTransformRot;}

//calculations
	bool checkDistance(const Vec3& point, const float& maxDistance) const{
		return fabsf(distanceToCylinderSurface(point)) < maxDistance;
	}
	bool checkNormal(const Vec3& normal, const Vec3& point, const float& mAngleEps) const{
		Vec3 transformedPoint;
		transformToCylinder(transformedPoint, point);
		Eigen::Vector2f dirFromCenter(transformedPoint.x(), transformedPoint.y());
		dirFromCenter.normalize();

		Vec3 normalCylinder = mTransformRot * normal;
		Eigen::Vector2f normal2 = normalCylinder.block<2,1>(0,0);

		return fabsf(dirFromCenter.dot(normal2)) >= cosf(mAngleEps);
	}
	void projectPoint(Vec3& outPoint, const Vec3& point) const{
		Vec3 transformedPoint;
		transformToCylinder(transformedPoint, point);
		Vec3 pOnAxisSameZ(0.f, 0.f, transformedPoint.z());
		Vec3 connection = transformedPoint - pOnAxisSameZ;
		float connectionLength = connection.norm();
		outPoint = pOnAxisSameZ + connection * (mRadius / connectionLength);
	}
	void transformToCylinder(Vec3& outPoint, const Vec3& point) const { outPoint = mTransformRot * (point + mTransformTrans);}
	void transformToXYZCoords(Vec3& outPoint, const Vec3& point) const { outPoint = (mReTransformRot * point) - mTransformTrans;}
	float distanceToAxis(const Vec3& point) const {
		Vec3 transformedPoint;
		transformToCylinder(transformedPoint, point);
		Eigen::Vector2f pointOnHeightZ(transformedPoint.x(), transformedPoint.y());
		return pointOnHeightZ.norm();
	}
	float distanceToCylinderSurface(const Vec3& point) const{
		return distanceToAxis(point) - mRadius;
	}
	void setTransformation(const float rotationAngle, const Vec3& rotationAxis){
		mTransformRot = Eigen::AngleAxisf(rotationAngle, rotationAxis);
		mReTransformRot = Eigen::AngleAxisf(-rotationAngle, rotationAxis);
		mTransformTrans = -mPointOnAxis;
	}

protected:
	/**
	 * \brief Calculate transformations from and to cylinder coordinate system.
	 */
	void computeTransformation(){
		Vec3 zAxis(0.f, 0.f, 1.f);
		calculateRotation(mRotAxis, mRotAngle, mAxisDirection, zAxis);
		mTransformRot = Eigen::AngleAxisf(mRotAngle, mRotAxis);
		mTransformTrans= -mPointOnAxis;
		mReTransformRot = Eigen::AngleAxisf(-mRotAngle, mRotAxis);
	}

	Vec3 mPointOnAxis;
	Vec3 mAxisDirection;
	float mRadius;

	Vec3 mRotAxis;
	float mRotAngle;
	Mat3 mTransformRot, mReTransformRot;
	Vec3 mTransformTrans;
};

#endif /* CYLINDER3D_H_ */
