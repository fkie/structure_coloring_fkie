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

#ifndef NODEPAIR_H_
#define NODEPAIR_H_

#include <structureColoring/OcTree.h>
#include <structureColoring/RotationHelper.h>

/**
 * \brief Pair of two nodes.
 */
class NodePair {
public:
	typedef Eigen::Vector3f Vec3;
	typedef OcTree::NodePtr NodePtr;

	NodePair(const NodePtr& nodeOne, const NodePtr& nodeTwo, const float& angleThreshold) :
		mOne(nodeOne), mTwo(nodeTwo), mStable(false){
		//mCross = mOne->value.normal.cross(mTwo->value.normal);
		float rotAngle;
		calculateRotation(mCross, rotAngle, mOne->value_.normal, mTwo->value_.normal);
		mNumPoints = mOne->value_.numPoints + mTwo->value_.numPoints;
		mCrossCurv = std::max(mOne->value_.curv, mTwo->value_.curv);
		mStable = true;//checkAngles(angleThreshold);
	}

	bool checkAngles(const float& angleThreshold) const {//TODO change to 2D ..
		Vec3 connectionOneTwo(mTwo->value_.meanPos - mOne->value_.meanPos);
		connectionOneTwo -= connectionOneTwo.dot( mCross ) * mCross;
		connectionOneTwo.normalize();
		return fabsf(mOne->value_.normal.dot(connectionOneTwo) - mTwo->value_.normal.dot(-connectionOneTwo)) > cos(angleThreshold);
	}

	bool stable() const {return mStable;}
	const NodePtr& getFirst() const {return mOne;}
	const NodePtr& getSecond() const {return mTwo;}
	const Vec3& getCross() const {return mCross;}
	float getCrossCurv() const {return mCrossCurv;}
	const unsigned int& getNumPoints() const {return mNumPoints;}
private:
	NodePtr mOne;
	NodePtr mTwo;
	Vec3 mCross;
	float mCrossCurv;
	unsigned int mNumPoints;
	bool mStable;
};
typedef std::vector<NodePair> NodePairs;

/**
 * \brief Position and normals of one pair of nodes.
 */
class NodePair2D {
public:
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Matrix3f Mat3;
	typedef Eigen::Vector2f Vec2;

	static void get2DTransformation(Mat3& trans, float& angle, Vec3& axis, const Vec3& cylinderDirectionHypothesis){
		const Vec3 zAxis(0.f, 0.f, 1.f);
		calculateRotation(axis, angle, zAxis, cylinderDirectionHypothesis);
		angle *= -1.f;
		trans = Eigen::AngleAxisf(angle, axis);
	}

	NodePair2D(const NodePair& nodePair, const Mat3& transformation, const float& dist): mDist(dist) {
		Vec3 posOne3 = transformation * nodePair.getFirst()->value_.meanPos;
		mPosOne = Vec2(posOne3.x(), posOne3.y());
		Vec3 normalOne3 = transformation * nodePair.getFirst()->value_.normal;
		mNormalOne = Vec2(normalOne3.x(), normalOne3.y());
		mNormalOne.normalize();
		Vec3 posTwo3 = transformation * nodePair.getSecond()->value_.meanPos;
		mPosTwo = Vec2(posTwo3.x(), posTwo3.y());
		Vec3 normalTwo3 = transformation * nodePair.getSecond()->value_.normal;
		mNormalTwo = Vec2(normalTwo3.x(), normalTwo3.y());
		mNormalTwo.normalize();
//		std::cout << "position1:" << std::endl << mPosOne << "normal1:" << std::endl << mNormalOne << std::endl;
//		std::cout << "position2:" << std::endl << mPosTwo << "normal2:" << std::endl << mNormalTwo << std::endl;
		initialize();
	}

	/**
	 * \brief Contructor, point pair with normals. Calculates circle mid-point and radius.
	 * \param nodeOnePosition First points position.
	 * \param nodeOneNormal First points normal.
	 * \param nodeTwoPosition Second points position.
	 * \param nodeTwoNormal Second points normal.
	 */
	NodePair2D(const Vec2& nodeOnePosition, const Vec2& nodeOneNormal,
			const Vec2& nodeTwoPosition, const Vec2& nodeTwoNormal, const float& dist) :
				mPosOne(nodeOnePosition), mNormalOne(nodeOneNormal), mPosTwo(
				nodeTwoPosition), mNormalTwo(nodeTwoNormal), mDist(dist) {
		initialize();
	}

	/**
	 * \brief Get stable condition.
	 */
	bool stable(){return mStable;}

	/**
	 * \brief Mid-point vector getter.
	 */
	const Vec2& getCircleMidPoint() const {return mCircleMidPoint;}

	/**
	 * \brief Radius getter.
	 */
	const float& getRadius() const {return mRadius;}
private:
	void initialize(){
		// set mOnePos + s1 * mOneNormal = mTwoPos + s2 * mTwoNormal
		// calculate s1, s2 -> get x, y
		float numerator = mNormalOne.y() * (mPosOne.x() - mPosTwo.x()) + mNormalOne.x() * (mPosTwo.y() - mPosOne.y());
		float denominator = mNormalTwo.x() * mNormalOne.y() - mNormalOne.x() * mNormalTwo.y();
		float s2 = numerator / denominator;
		mCircleMidPoint.x() = mPosTwo.x() +  s2 * mNormalTwo.x();
		mCircleMidPoint.y() = mPosTwo.y() + s2 * mNormalTwo.y();
		mRadius = (distToMidPoint(mPosOne) + distToMidPoint(mPosTwo)) * 0.5f;
		mStable = checkRadius();
	}

	bool checkRadius(){
		return fabsf(distToMidPoint(mPosOne) - mRadius) < mDist && std::isfinite(mCircleMidPoint.y()) && std::isfinite(mCircleMidPoint.x());
	}

	/**
	 * \brief Calculate distance from mid-point to given position.
	 * \param vec Input vector (2D point) for distance meassurement.
	 */
	float distToMidPoint(const Vec2& vec) const {
		Vec2 connectingVec(mCircleMidPoint - vec);
		return connectingVec.norm();
	}

	Vec2 mPosOne;
	Vec2 mNormalOne;

	Vec2 mPosTwo;
	Vec2 mNormalTwo;

	float mDist;

	float mRadius;
	Vec2 mCircleMidPoint;
	bool mStable;
};
typedef std::vector<NodePair2D> NodePairs2D;


#endif /* NODEPAIR_H_ */
