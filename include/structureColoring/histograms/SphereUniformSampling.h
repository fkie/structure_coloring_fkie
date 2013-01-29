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

#ifndef SPHEREUNIFORMSAMPLING_H_
#define SPHEREUNIFORMSAMPLING_H_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef std::pair<unsigned int, unsigned int> Pair2ui;

class SphereUniformSampling {
public:
	class NormalEntry{
	public:
		NormalEntry(const unsigned int i, const unsigned int j, const unsigned int phi_resolution);

		float getWeight() const{return mWeight;}
		const Eigen::Vector3f& getNormal() const{return mNormal;}
		void getAngles(float& phi, float& theta) const{phi=mPhi; theta=mTheta;}
		const std::list<int>& getPoints() const{return mPoints;}
		const std::list<float>& getWeights() const{return mWeights;}

		void addIndex(std::list<int>::iterator& points_it, std::list<float>::iterator& weights_it, const int idx, const float w);
		void deletePointNWeight(const std::list<int>::iterator points_it, const std::list<float>::iterator weights_it);
		void deleteEntry(){mWeight = 0.f; mPoints.clear(); mWeights.clear();}

	private:
		static void getNormalFromAngles(Eigen::Vector3f& normal, const float phi, const float theta);

		float mWeight;
		Eigen::Vector3f mNormal;
		float mPhi, mTheta;
		std::list<int> mPoints;
		std::list<float> mWeights;
	};

	SphereUniformSampling(unsigned int phi_resolution, bool halfSphereFlip = false);

	static void getHistogramIndicesFromAngles(unsigned int& i, unsigned int& j, const float phi, const float theta, const unsigned int phi_resolution);
	static void getAnglesFromHistogramIndices(float& phi, float& theta, const unsigned int i, const unsigned int j, const unsigned int phi_resolution);

	Eigen::Vector3f getNormal(const unsigned int i, const unsigned int j) const{if ((i<mData.size())&&(j<mData[i].size())) return mData[i][j].getNormal(); return Eigen::Vector3f(0, 0, 0);}
	void getPointsAngle(std::list<int>& points, const float phi, const float theta, const float max_angle) const;
	void getPoints(std::vector<int>& points, const unsigned int i, const unsigned int j, std::vector<float>* weights = NULL) const;
	float getMaximumAngle(float& phi, float& theta, const float max_angle) const;
	float getMaximumIndices(unsigned int& idx_i, unsigned int& idx_j, const float max_angle) const;
	float getMaximum(unsigned int& imax, unsigned int& jmax) const;
	void getAllAngleNeighbors(std::vector<Pair2ui>& neighbors, const float phi, const float theta, const float max_angle) const;
	void getAllAngleNeighbors(std::vector<Pair2ui>& neighbors, const Eigen::Vector3f& pointNormal, const float max_angle) const;
	void getHistogramAsPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& histogramCloud, const float scale, const Eigen::Vector3f& translation) const;

//	void addPoint(const float phi, const float theta, const int index, const float max_angle_tolerance = 0.f, const float weight = 1.f);
//	void addPoint(const Eigen::Vector3f pointNormal, const int index, const float max_angle_tolerance = 0.f, const float weight = 1.f);
	void addPointNoNeighbors(std::list<int>::iterator& points_it, std::list<float>::iterator& weights_it, const unsigned int i, const unsigned int j, const int index, const float weight = 1.f);
	void deletePointsAtAngle(const float phi, const float theta, const float max_angle);
	void deletePointNWeightAt(const std::list<int>::iterator points_it, const std::list<float>::iterator weights_it, const unsigned int i, const unsigned int j);

private:
	float getWeightAngle(const float phi, const float theta, const float max_angle) const;
	float getPlanarAngle(const float phi1, const float theta1, const float phi2, const float theta2) const;

	void deleteEntry(unsigned int i, unsigned int j);

	unsigned int mPhi_resolution;
	bool mHalfSphereFlip;
	std::vector<std::vector<NormalEntry> > mData;
	std::vector<float> mRatioToNextData;
};

#endif /* SPHEREUNIFORMSAMPLING_H_ */
