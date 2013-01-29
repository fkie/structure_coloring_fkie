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

#include <structureColoring/histograms/SphereUniformSampling.h>

SphereUniformSampling::SphereUniformSampling(unsigned int phi_resolution, bool halfSphereFlip)
	:mPhi_resolution(phi_resolution), mHalfSphereFlip(halfSphereFlip){
	mData.resize(mPhi_resolution);
	mRatioToNextData.resize(mPhi_resolution - 1);
	for(unsigned int i=0; i<mPhi_resolution; i++){
		unsigned int uisize = 2 * mPhi_resolution * sin (M_PI * (float)i / (float)mPhi_resolution) + 1.5;
		mData[i].reserve(uisize);
		for(unsigned int j=0; j<uisize; j++){
			mData[i].push_back(SphereUniformSampling::NormalEntry(i, j, mPhi_resolution));
		}
		if(i>0)mRatioToNextData[i-1] = uisize / mData[i-1].size();
	}
}

//void SphereUniformSampling::mergeVectors(std::vector<int>& out, const std::vector<int>& in1, const std::vector<int>& in2){
//	if(in1.size() == 0) out = in2;
//	else if(in2.size() == 0) out = in1;
//	else{
//		out.clear();
//		out.reserve(in1.size() + in2.size());
//		unsigned int j=0;
//		for(unsigned int i=0;i<in1.size(); i++){
//			while ((j<in2.size())&&(in1[i] > in2[j])){
//				out.push_back(in2[j]);
//				j++;
//			}
//			if (in1[i] < in2[j])out.push_back(in1[i]);
//		}
//		for (;j<in2.size();j++){
//			out.push_back(in2[j]);
//		}
//	}
//}

void SphereUniformSampling::getHistogramIndicesFromAngles(unsigned int& i, unsigned int& j, const float phi, const float theta, const unsigned int phi_resolution){
	float phi_corrected = phi;
	if (phi_corrected > M_PI){
		phi_corrected = M_PI;
		ROS_ERROR("getHistogramIndicesFromAngles: phi (%f) > PI", phi);
	}
	i = ((phi_corrected/(M_PI)) * phi_resolution)+0.5;

	float theta_corrected = theta;
	if (theta_corrected >= 2.f * M_PI){
		theta_corrected = 0;
		ROS_ERROR("getHistogramIndicesFromAngles: theta (%f) > 2 * PI", theta);
	}
	unsigned int uisize = 2 * phi_resolution * sin (M_PI * (float)i / (float)phi_resolution) + 1.5;
	j = ((theta_corrected / (2.f * M_PI)) * uisize)+0.5;
}

void SphereUniformSampling::getAnglesFromHistogramIndices(float& phi, float& theta, const unsigned int i, const unsigned int j, const unsigned int phi_resolution){
	unsigned int uisize = 2 * phi_resolution * sin (M_PI * (float)i / (float)phi_resolution) + 1.5;
	if((i==(unsigned int)-1)||(j==(unsigned int)-1)){
		phi = -M_PI;
		theta = -M_PI;
	} else {
		phi = (float)i * M_PI / (float)phi_resolution;
		theta = 2.f * M_PI * ((float)(j) / ((float)uisize));
	}
	if (phi > M_PI){
		if (phi < M_PI +0.01){
			phi = M_PI - 0.0001;
		} else {
			ROS_ERROR("getAnglesFromHistogramIndices: phi (%f) > PI", phi);
			phi = M_PI;
		}
	}
	if (theta >= 2 * M_PI){
		if (theta < 2 * M_PI +0.01){
			theta = 2 * M_PI - 0.0001;
		} else {
			ROS_ERROR("getAnglesFromHistogramIndices: theta (%f) >= 2 * PI", theta);
			theta = 0;
		}
	}
}

void SphereUniformSampling::getPointsAngle(std::list<int>& points, const float phi, const float theta, const float max_angle) const{
	points.clear();
	std::list<int> nPoints;
	if((phi >= 0)&&(theta >= 0)){
		std::vector<Pair2ui> neighbors;
		getAllAngleNeighbors(neighbors, phi, theta, max_angle);
		for(unsigned int n=0; n<neighbors.size(); n++){
			nPoints = mData[neighbors[n].first][neighbors[n].second].getPoints();
			points.merge(nPoints);
		}
	}
}

void SphereUniformSampling::getPoints(std::vector<int>& points, const unsigned int i, const unsigned int j, std::vector<float>* weights) const{
	points.clear();
	if ((i < mData.size()) && (j < mData[i].size())){
		const std::list<int>& points_list = mData[i][j].getPoints();
		for(std::list<int>::const_reverse_iterator list_it=points_list.rbegin(); list_it != points_list.rend(); ++list_it){
			points.push_back(*list_it);
		}
		if (weights != NULL){
			const std::list<float>& weights_list = mData[i][j].getWeights();
			for(std::list<float>::const_reverse_iterator list_it=weights_list.rbegin(); list_it != weights_list.rend(); ++list_it){
				weights->push_back(*list_it);
			}
		}
	}
}

float SphereUniformSampling::getMaximumAngle(float& phi, float& theta, const float max_angle) const{
	float max_weight=-1.f, phi_ij, theta_ij;
	unsigned int max_i=-1, max_j=-1;
	for(unsigned int i=0; i<mData.size(); i++){
		for(unsigned int j=0; j<mData[i].size(); j++){
			mData[i][j].getAngles(phi_ij, theta_ij);
			float tmp_weight = getWeightAngle(phi_ij, theta_ij, max_angle);
			if (tmp_weight > max_weight){
				max_weight = tmp_weight;
				max_i = i;
				max_j = j;
			}
		}
	}
	mData[max_i][max_j].getAngles(phi, theta);
	return max_weight;
}

float SphereUniformSampling::getMaximumIndices(unsigned int& idx_i, unsigned int& idx_j, const float max_angle) const{
	float max_weight=-1.f, phi_ij, theta_ij;
	unsigned int max_i=-1, max_j=-1;
	for(unsigned int i=0; i<mData.size(); i++){
		for(unsigned int j=0; j<mData[i].size(); j++){
			mData[i][j].getAngles(phi_ij, theta_ij);
			float tmp_weight = getWeightAngle(phi_ij, theta_ij, max_angle);
			if (tmp_weight > max_weight){
				max_weight = tmp_weight;
				max_i = i;
				max_j = j;
			}
		}
	}
	idx_i = max_i;
	idx_j = max_j;
	return max_weight;
}

float SphereUniformSampling::getMaximum(unsigned int& imax, unsigned int& jmax) const{
	imax = 0; jmax = 0;
	float wmax = 0.f;
	for (unsigned int i=0; i<mData.size(); i++){
		for(unsigned int j=0; j<mData[i].size(); j++){
			float w = mData[i][j].getWeight();
			if(w > wmax){
				imax = i;
				jmax = j;
				wmax = w;
			}
		}
	}
	return wmax;
}

void SphereUniformSampling::getAllAngleNeighbors(std::vector<Pair2ui>& neighbors, const float phi, const float theta, const float max_angle) const{
	float dotProduct = 0.f, max_angle_cos = cos(max_angle);
	neighbors.clear();
	Eigen::Vector3f pointNormal(0, 0, 0);
	Eigen::Vector3f binNormal;
	pointNormal.z() = cos(phi);
	pointNormal.y() = sin(theta) * sin(phi);
	pointNormal.x() = cos(theta) * sin(phi);
	for(unsigned int i=0; i<mData.size(); i++){
		for(unsigned int j=0; j<mData[i].size(); j++){
			binNormal = mData[i][j].getNormal();
			dotProduct = binNormal.dot(pointNormal);
			if(mHalfSphereFlip) dotProduct = fabsf(dotProduct);
			if (dotProduct >= max_angle_cos){
				neighbors.push_back(std::make_pair(i, j));
			}
		}
	}
}

void SphereUniformSampling::getAllAngleNeighbors(std::vector<Pair2ui>& neighbors, const Eigen::Vector3f& pointNormal, const float max_angle) const{
	float dotProduct = 0.f, max_angle_cos = cos(max_angle);
	neighbors.clear();
	Eigen::Vector3f binNormal;
	for(unsigned int i=0; i<mData.size(); i++){
		for(unsigned int j=0; j<mData[i].size(); j++){
			binNormal = mData[i][j].getNormal();
			dotProduct = binNormal.dot(pointNormal);
			if(mHalfSphereFlip) dotProduct = fabsf(dotProduct);
			if (dotProduct >= max_angle_cos){
				neighbors.push_back(std::make_pair(i, j));
			}
		}
	}
}

void SphereUniformSampling::getHistogramAsPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& histogramCloud, const float scale, const Eigen::Vector3f& translation) const{
	pcl::PointXYZRGB binNormal;
	unsigned int imax=0, jmax=0;
	getMaximum(imax, jmax);
	float maxWeight = mData[imax][jmax].getWeight();
	for (unsigned int i=0; i<mData.size(); i++){
		for (unsigned int j=0; j<mData[i].size(); j++){
			Eigen::Vector3f normal = mData[i][j].getNormal();
			binNormal.x = normal.x() * scale + translation.x();
			binNormal.y = normal.y() * scale + translation.y();
			binNormal.z = normal.z() * scale + translation.z();
			//unsigned int rgb = 0xffffff;
			unsigned int rgb = 0x000000;
//			if ((i == imax) && (j == jmax)) rgb = 0x00ffff;
//			else {
				unsigned int w = (unsigned int)(255 * mData[i][j].getWeight() / maxWeight);//(getWeightAngle(phi, theta, max_angle)+0.5);
				if(w > 0xff) w = 0xff;
				unsigned int col = (w<<8) + (w<<16) + w;
				if(col > 0xffffff)col = 0xffffff;
				//rgb -= col;
				rgb += col;
//			}
			binNormal.rgb = *(float*)&rgb;
			histogramCloud.points.push_back(binNormal);
		}
	}
}

void SphereUniformSampling::addPointNoNeighbors(std::list<int>::iterator& points_it, std::list<float>::iterator& weights_it, const unsigned int i, const unsigned int j, const int index, const float weight){
	if ((i<mData.size()) && (j<mData[i].size()))
		mData[i][j].addIndex(points_it, weights_it, index, weight);
}

void SphereUniformSampling::deletePointsAtAngle(const float phi, const float theta, const float max_angle){
	std::vector<Pair2ui> neighbors;
	getAllAngleNeighbors(neighbors, phi, theta, max_angle);
	for(unsigned int n=0; n<neighbors.size(); n++){
		deleteEntry(neighbors[n].first, neighbors[n].second);
	}
}

void SphereUniformSampling::deletePointNWeightAt(const std::list<int>::iterator points_it, const std::list<float>::iterator weights_it, const unsigned int i, const unsigned int j){
	if ((i<mData.size()) && (j<mData[i].size())){
		mData[i][j].deletePointNWeight(points_it, weights_it);
	}
}

float SphereUniformSampling::getWeightAngle(const float phi, const float theta, const float max_angle) const{
	float ret_weight=0.f;
	std::vector<Pair2ui> neighbors;
	getAllAngleNeighbors(neighbors, phi, theta, max_angle);
	for(unsigned int n=0; n<neighbors.size(); n++){
		ret_weight += mData[neighbors[n].first][neighbors[n].second].getWeight();
	}
	return ret_weight;
}

float SphereUniformSampling::getPlanarAngle(const float phi1, const float theta1, const float phi2, const float theta2) const{
	Eigen::Vector3f normal1(cos(phi1)*sin(theta1),sin(phi1)*sin(theta1),cos(theta1));
	Eigen::Vector3f normal2(cos(phi2)*sin(theta2),sin(phi2)*sin(theta2),cos(theta2));
	return acos(normal1.dot(normal2));
}

void SphereUniformSampling::deleteEntry(unsigned int i, unsigned int j){
	if ((i < mData.size()) && (j < mData[i].size())){
		mData[i][j].deleteEntry();
	}
}

SphereUniformSampling::NormalEntry::NormalEntry(const unsigned int i, const unsigned int j, const unsigned int phi_resolution){
	SphereUniformSampling::getAnglesFromHistogramIndices(mPhi, mTheta, i, j, phi_resolution);
	getNormalFromAngles(mNormal, mPhi, mTheta);
	mWeight = 0;
}

void SphereUniformSampling::NormalEntry::addIndex(std::list<int>::iterator& points_it, std::list<float>::iterator& weights_it, const int idx, const float w){
	mWeight+=w;
	mPoints.push_back(idx);
	mWeights.push_back(w);
	points_it = mPoints.end();
	points_it--;
	weights_it = mWeights.end();
	weights_it--;
}

void SphereUniformSampling::NormalEntry::deletePointNWeight(const std::list<int>::iterator points_it, const std::list<float>::iterator weights_it){
	mPoints.erase(points_it);
	mWeight -= *weights_it;
	mWeights.erase(weights_it);
}

void SphereUniformSampling::NormalEntry::getNormalFromAngles(Eigen::Vector3f& normal, const float phi, const float theta){
	normal.z() = cos(phi);
	normal.y() = sin(theta) * sin(phi);
	normal.x() = cos(theta) * sin(phi);
}
