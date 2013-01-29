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

#ifndef TRIANGLEDISTRIBUTIONHISTOGRAMWITHREMOVE_H_
#define TRIANGLEDISTRIBUTIONHISTOGRAMWITHREMOVE_H_

#include "TriangleDistributionHistogram.h"

class TriangleDistributionHistogramWithRemove : public TriangleDistributionHistogram{
public:
	TriangleDistributionHistogramWithRemove(const float& min, const float& max, const float& binSize, const float& deviation,
			const size_t& maxIndex)
		: TriangleDistributionHistogram(min, max, binSize, deviation, maxIndex), mIndexLUT(LUTType(maxIndex)){}

	void addDistributedToBins(const float& key, const WeightedIdx& value);
	void removeDistributed(const IdxNWeight& wi, const int& index);
	void removeAllFromBin(const float& key);

private:
	typedef std::vector<IdxNWeight> IdxWeightPairs;
	typedef std::vector<IdxWeightPairs> LUTType;

	LUTType mIndexLUT;
};

inline void TriangleDistributionHistogramWithRemove::addDistributedToBins(const float& key, const WeightedIdx& value){
	assert(key > mMin);
	assert(key < mMax);
	int startI = std::max( 0, (int) floor( ((key - mDev) - mMin) / mBinSize ) );
	int stopI = std::min( (int)mBins.size(), (int) ceil( ((key + mDev) - mMin) / mBinSize ) );
	for(int i = startI; i < stopI; ++i){
		float loopKey = getKey(i);
		float scale = (mDev - fabsf(loopKey - key)) / mDev;
		if(scale > 0.f){
			WeightedIdx loopValue(value);
			loopValue.weight() *= scale;
			mBins[i] += loopValue;
			IdxWeightPairs& LUTEntry(mIndexLUT[loopValue.index()]);
			if(LUTEntry.size() == LUTEntry.capacity()-1)
				LUTEntry.reserve(LUTEntry.size() * 100);
			LUTEntry.push_back(std::make_pair(i, loopValue.weight()));
		}
	}
}

inline void TriangleDistributionHistogramWithRemove::removeDistributed(const IdxNWeight& inw, const int& index){
	mBins[inw.first] -= WeightedIdx(inw.second, index);
}

inline void TriangleDistributionHistogramWithRemove::removeAllFromBin(const float& key){
	if(key < mMin || key > mMax){
		char exchar[255];
		sprintf(exchar, "%f < %f < %f", mMin, key, mMax);
		std::string exceptiontext(exchar);
		throw OutOfRange(exceptiontext);
	}
	typedef std::vector<int> IdxVector;
	IdxVector nodes(mBins[getIdx(key)].nodeIndices());
	for(IdxVector::const_iterator bin_it = nodes.begin(); bin_it != nodes.end(); ++bin_it){
		for(IdxWeightPairs::const_iterator lut_it = mIndexLUT[*bin_it].begin(); lut_it != mIndexLUT[*bin_it].end(); ++lut_it){
			removeDistributed(*lut_it, *bin_it);
		}
		mIndexLUT[*bin_it].clear();
	}
}

#endif /* TRIANGLEDISTRIBUTIONHISTOGRAMWITHREMOVE_H_ */
