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

#ifndef TRIANGLEDISTRIBUTIONHISTOGRAM2D_H_
#define TRIANGLEDISTRIBUTIONHISTOGRAM2D_H_

#include <structureColoring/histograms/Histogram2D.h>
#include <structureColoring/histograms/WeightedIdxVector.h>
#include <assert.h>

class TriangleDistributionHistogram2D : public Histogram2D<float, WeightedIdx, WeightedIdxVector> {
public:
	typedef std::pair<float, float> Keys;

	TriangleDistributionHistogram2D(const float& minX, const float& maxX, const float& minY, const float& maxY,
			const float& binSize, const float& deviation) :
		Histogram2D<float, WeightedIdx, WeightedIdxVector> (minX, maxX, minY, maxY, binSize), mDev(deviation) {
	}

	void addDistributedToBins(const float& xKey, const float& yKey, const WeightedIdx& value) {
		addDistributedToBins(std::make_pair(xKey, yKey), value);
	}

	void addDistributedToBins(const Keys& keys, const WeightedIdx& value){
		assert(keys.first >= mMinX);
		assert(keys.first <= mMaxX);
		assert(keys.second >= mMinY);
		assert(keys.second <= mMaxY);
		int startIdxX, startIdxY, stopIdxX, stopIdxY;
		startIdxX = std::max( 0, (int) floor( ((keys.first - mDev) - mMinX) / mBinSize ) );
		startIdxY = std::max( 0, (int) floor( ((keys.second - mDev) - mMinY) / mBinSize ) );
		stopIdxX = std::min( (int)mXSpan, (int) ceil( ((keys.first + mDev) - mMinX) / mBinSize ) );
		stopIdxY = std::min( (int)mYSpan, (int) ceil( ((keys.second + mDev) - mMinY) / mBinSize ) );
//		std::cout << "startX = " << startIdxX << "; stopX = " << stopIdxX << "; startY = "
//									<< startIdxY << "; stopY = " << stopIdxY << "," << std::endl;
		for(int x = startIdxX; x < stopIdxX; ++x){
			for(int y = startIdxY; y < stopIdxY; ++y){
				size_t loopIndex = getIdx((size_t)x,(size_t) y);
				Keys loopKeys = getKeys(loopIndex);
//				std::cout << "keys = (" << keys.first << ", " << keys.second << "); loopkeys = (" << loopKeys.first << ", " << loopKeys.second << ");" << std::endl;
				float dist2d = sqrt((keys.first - loopKeys.first) * (keys.first - loopKeys.first)
						+ (keys.second - loopKeys.second) * (keys.second - loopKeys.second));
				float scale = (mDev - dist2d) / mDev;
//				std::cout << "x = " << x << "; y = " << y << "; loopIndex = " << loopIndex << "; maxIndex = " << mBins.size() << "; scale = " << scale << ";" << std::endl;
				if (scale > 0.f){
					WeightedIdx loopValue(value);
					loopValue.weight() *= scale;
					mBins[loopIndex] += loopValue;
//					std::cout << "Histogram2D, TriangleDistribution, added weight (" << loopValue.weight() << ") to bin with index "
//							<< loopIndex << " at keys (" << x << ", " << y << ")"<< std::endl;
				}
			}
		}
	}

	float deviation() const {
		return mDev;
	}

	float& deviation() {
		return mDev;
	}
private:
	float mDev;
};

#endif /* TRIANGLEDISTRIBUTIONHISTOGRAM2D_H_ */
