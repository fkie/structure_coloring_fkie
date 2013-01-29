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

#ifndef TRIANGLEDISTRIBUTIONHISTOGRAM_H_
#define TRIANGLEDISTRIBUTIONHISTOGRAM_H_

#include <structureColoring/histograms/Histogram.h>
#include <structureColoring/histograms/WeightedIdxVector.h>
#include <assert.h>

class TriangleDistributionHistogram : public Histogram<float, WeightedIdx, WeightedIdxVector>{
public:
	TriangleDistributionHistogram(const float& min, const float& max, const float& binSize, const float& deviation, const size_t& maxIndex)
		: Histogram<float, WeightedIdx, WeightedIdxVector>(min, max, binSize), mDev(deviation), mMaxIndex(maxIndex) {}

	typedef std::pair<int, float> IdxNWeight;

	void addDistributedToBins(const float& key, const WeightedIdx& value);

	float deviation() const {return mDev;}
	float& deviation() {return mDev;}
protected:
	float mDev;
	size_t mMaxIndex;
};

inline void TriangleDistributionHistogram::addDistributedToBins(const float& key, const WeightedIdx& value){
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
		}
	}
}

#endif /* TRIANGLEDISTRIBUTIONHISTOGRAM_H_ */
