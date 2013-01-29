/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Torsten Fiolka, Bastian Gaspers
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

#ifndef NORMAL_ESTIMATION_VALUE_CLASS_H_
#define NORMAL_ESTIMATION_VALUE_CLASS_H_

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <octreelib/spatialaggregate/octree.h>
#include <octreelib/algorithm/downsample.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/vector_average.h>
#include <limits.h>

class NormalEstimationValueClass {
public:
	typedef Eigen::Matrix3f Mat3;
	typedef Eigen::Vector3f Vec3;

	Mat3 summedSquares;
	Vec3 summedPos;
	Vec3 meanPos;

	Vec3 normal;
	float curv;
	float curv2;
	bool stable;
	bool segmented;
	unsigned int id;
	unsigned int numPoints;
	bool nodeQuerried;

	NormalEstimationValueClass(const unsigned int) :
		id(std::numeric_limits<unsigned int>::max()) {
		clear();
	}

	NormalEstimationValueClass() :
		id(std::numeric_limits<unsigned int>::max()) {
		clear();
	}

	void clear() {
		summedSquares.setZero();
		summedPos.setZero();
		meanPos.setZero();
		normal.setZero();
		numPoints = 0;
		curv = -1.f;
		curv2 = -1.f;
		segmented = false;
		stable = false;
		nodeQuerried = false;
	}

	NormalEstimationValueClass& operator+(const NormalEstimationValueClass& rhs) {
		summedSquares += rhs.summedSquares;
		summedPos += rhs.summedPos;
		numPoints += rhs.numPoints;
		if (rhs.id < id)
			id = rhs.id;
		return *this;
	}

	NormalEstimationValueClass& operator+=(const NormalEstimationValueClass& rhs) {
		summedSquares += rhs.summedSquares;
		summedPos += rhs.summedPos;
		numPoints += rhs.numPoints;
		if (rhs.id < id)
			id = rhs.id;
		return *this;
	}
};

//! Gets the normal of a given integral value and a point count
template<typename CoordType, typename ValueType>
bool calculateNormal(spatialaggregate::OcTreeNode<CoordType, ValueType>* treenode, const Eigen::Vector3f& viewPort,
		const int& minimumPointsForNormal, const float& curv_threshold, const float& curv2_threshold,
		const float& minPrincipleVariance) {
	int count = treenode->value_.numPoints;
	Eigen::Vector3f summedPos(treenode->value_.summedPos);
	summedPos *= 1.f / (float) count;
	treenode->value_.meanPos = summedPos;
	if (count < minimumPointsForNormal) {
		return false;
	}

	Eigen::Matrix3f summedSquares(treenode->value_.summedSquares);
	summedSquares *= 1.f / (float) count;
	summedSquares -= summedPos * summedPos.transpose();

	//Eigen::Matrix<float, 3, 1> eigen_values;
	//Eigen::Matrix<float, 3, 3> eigen_vectors;
	//pcl::eigen33(summedSquares, eigen_vectors, eigen_values);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
	es.compute(summedSquares);
	const Eigen::Matrix<float, 3, 1>& eigen_values = es.eigenvalues();
	const Eigen::Matrix<float, 3, 3>& eigen_vectors = es.eigenvectors();

	treenode->value_.curv = eigen_values(0, 0) / eigen_values(2, 0);//(eigen_values(0, 0) + eigen_values(1, 0) + eigen_values(2, 0));
	if(eigen_values(0, 0) > eigen_values(2, 0)) std::cout << "ERROR while calculating curv: eigenvalues were not sorted ascendingly" << std::endl;
	treenode->value_.curv2 = eigen_values(1, 0) / eigen_values(2, 0);
	treenode->value_.normal = eigen_vectors.col(0);
	if (eigen_vectors.col(0).dot((summedPos / treenode->value_.numPoints) - viewPort) > 0)
		treenode->value_.normal = -treenode->value_.normal;
	if ((treenode->value_.curv > curv_threshold) || (treenode->value_.curv2 < curv2_threshold)
			|| (eigen_values(2, 0) < minPrincipleVariance)) {
		treenode->value_.stable = false;
		return false;
	}
	treenode->value_.stable = true;
	return true;
}

template<typename CoordType, typename ValueType>
void calculateNormalsOnOctreeLayer(std::vector<spatialaggregate::OcTreeNode<CoordType, ValueType>*>& layer,
		Eigen::Vector3f viewPort, int minimumPointsForNormal) {
	for (unsigned int i = 0; i < layer.size(); i++) {
		if (!(layer[i]->value_.stable)) {
			layer[i]->value_.stable = calculateNormal(layer[i], layer[i]->value_.normal, viewPort,
					minimumPointsForNormal);
		}
	}
}

#endif /* NORMAL_ESTIMATION_VALUE_CLASS_H_ */

