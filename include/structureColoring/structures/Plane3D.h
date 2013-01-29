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

#ifndef PLANE_H_
#define PLANE_H_

#include <structureColoring/RotationHelper.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

class Plane3D{
public:
	typedef Eigen::Vector3f Vec3;
	typedef boost::shared_ptr<Plane3D> Plane3DPtr;

	Plane3D() : mNormal(0.f,0.f,0.f), mDistanceToOrigin(0) {}
	Plane3D(const Vec3& normal, const float distanceToOrigin) : mNormal(normal), mDistanceToOrigin(distanceToOrigin) {
		initializeTransformation();
	}

//getter
	const Vec3& getPlaneNormal() const {
		return mNormal;
	}
	const float& getPlaneDistanceToOrigin() const {
		return mDistanceToOrigin;
	}

//setter
	void setPlaneNormal(const Vec3& normal){
		mNormal = normal;
	}
	void setPlaneDistanceToOrigin(const float& dist){
		mDistanceToOrigin = dist;
	}

//calculations
	bool checkDistance(const Vec3& p, const float& maxDistance) const {
		return (distance(p) <= maxDistance);
	}
	bool checkNormal(const Vec3& n, const float& maxAngle) const {
		return fabsf(mNormal.dot(n)) > cos(maxAngle);
	}
	float distance(const Vec3& p) const {
		return fabsf(signedDistance(p));
	}
	float signedDistance(const Vec3& p) const {
		return mNormal.dot(p) - mDistanceToOrigin;
	}
	Vec3 getOrthogonalProjectionOntoPlane(const Vec3& p) const {
		return p - mNormal * signedDistance(p);
	}
	Vec3 transformToXYPlane(const Vec3& p) const {
		return (mPcs * (p - mNormal * mDistanceToOrigin));
	}
	Vec3 transformToXYZCoords(const Vec3& p) const {
		return (mPcst * p) + mNormal * mDistanceToOrigin;
	}

protected:
	void initializeTransformation(){
		Vec3 coordAxis = Vec3(0, 0, 1);
		Vec3 rotAxis;
		float rotAngle;
		calculateRotation(rotAxis, rotAngle, coordAxis, mNormal);
		mPcs = Eigen::AngleAxisf(-rotAngle, rotAxis);
		mPcst = mPcs.transpose();
	}

	Vec3 mNormal;
	float mDistanceToOrigin;

//Transformation
	Eigen::Matrix3f mPcs, mPcst;
};

#endif /* PLANE_H_ */
