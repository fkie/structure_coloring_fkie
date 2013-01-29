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

#ifndef STRCOLPARAMS_H_
#define STRCOLPARAMS_H_

#include <string>

class StrColParams {
public:
	StrColParams();
	virtual ~StrColParams();

	void parseParamFile(std::string filename);

private:
	void init();

public:
//params for computation of segmentation
	float mTexelSizeFactor;
	unsigned int mDilateIterations;
	unsigned int mPhi_resolution; //number of steps from 0 to PI
	float mMaxSACDistanceThreshold;
	float mMinSACDistanceThreshold;
	float mSACOctreeFactor;
	float mMaxHTDistanceThreshold;
	float mHTOctreeBinSizeFactor;
	float mHTDistanceDeviationFactor;
	float mAngleEps; //angle epsilon in radians - angle between one gridcellnormal and planenormal may differ no more than angle epsilon
	float mAngleEpsOnMinOctreeRes;
	float mSACOutlierRatioThreshold;
	unsigned int mMinPointsInCC; //min number of points in a connected component
	unsigned int mMinNodesInCC;
	unsigned int mMinNodesInCylinder;
	float mMergePlanesSimilarityFactor;
	float mNodeToBBDistance;
	int mDebugSteps;
	int mConnectionNeighbors;
	float mRho_max;
	int mMinPointsForNormal;
	float mCurvThreshold;
	float mCurv2Threshold;
	float mPrincipalVarFactor;
	unsigned int mMinOctreeNodes;
	unsigned int mTriesOnEachDepth;
	float mMinOctreeResolution;
	float mSqDistFactor;
	float mNodeSegmentedRatio;

//params for cylinders only
	float mNormalDistanceWeight;
	float mMinRadiusFactor;
	float mMaxRadiusFactor;
	float mMinCylinderRadius;
	float mMaxCylinderRadius;
	unsigned int mCylinderBins;
	unsigned int mCylinderPairNeighbors;
	float mMidPointBinSizeFactor;
	float mRadiusDevFactor;
	float mOccupiedRatio;
	float mCylinderHeightDev;

//params from command line input
	bool mVerbose;
	bool mPGM;
	bool mKinect;
	bool mLaser;
	bool mABW;
	bool mPerceptron;
	bool mTextures;
	bool mCylinder;
	bool mPclSAC;
	bool mNoRansacStep;
	bool mWriteRawPic;
	std::string mRawPicFilename;
	unsigned int mRawPicCounter;
	bool mLoadPCD;
	bool mPCDnoRGB;
	std::string mRuntimeFilename;
	std::string mLaserTopic;
	int mPclSACmaxIter;
	int mOnlyDepth;
	std::string mInputFilename, mOutputFilename, mNormalOutputFilename;

	int mPostProcessing;
};

#endif /* STRCOLPARAMS_H_ */
