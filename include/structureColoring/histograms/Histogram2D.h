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

#ifndef HISTOGRAM2D_H_
#define HISTOGRAM2D_H_

#include <functional>
#include <algorithm>
#include <vector>
#include <utility>
#include <structureColoring/histograms/OutOfRangeException.h>

template<class Key, class Value, class BinType = Value>
class Histogram2D {
public:
	typedef OutOfRangeException OutOfRange;
	typedef std::pair<Key, Key> Keys;

	/** \brief Constructor.
	 * \param min minimum key value (inclusive).
	 * \param max maximum key value (inclusive). The key max itself is allocated to the last bin.
	 * \param binSize the size of one bin.
	 */
	Histogram2D(const Key& minX, const Key& maxX, const Key& minY, const Key& maxY, const Key& binSize) :
		mMinX(minX), mMaxX(maxX), mMinY(minY), mMaxY(maxY), mBinSize(binSize),
		mXSpan(floor( (mMaxX - mMinX) / mBinSize ) + 1), mYSpan(floor( (mMaxY - mMinY) / mBinSize ) + 1),
		mBins(mXSpan * mYSpan){
	}

	Histogram2D(const Keys& min, const Keys& max, const Key& binSize) :
		mMinX(min.first), mMaxX(max.first), mMinY(min.second), mMaxY(max.second), mBinSize(binSize),
		mXSpan(floor( (mMaxX - mMinX) / mBinSize ) + 1), mYSpan(floor( (mMaxY - mMinY) / mBinSize ) + 1),
		mBins(mXSpan * mYSpan){
	}

	BinType& bin(const Key& x, const Key& y) { return mBins[getIdx(x, y)]; }
	const BinType& bin(const Key& x, const Key& y) const { return mBins[getIdx(x, y)]; }
	BinType& bin(const Keys& k) { return mBins[getIdx(k.first, k.second)]; }
	const BinType& bin(const Keys& k) const { return mBins[getIdx(k.first, k.second)]; }

	void addToBin(const Key& x, const Key& y, const Value& v) { mBins[getIdx(x, y)] += v; }
	void addToBin(const Keys& k, const Value& v) { mBins[getIdx(k.first, k.second)] += v; }

	void getMaxBin(BinType& bin, Key& x, Key& y) const {
		Keys k;
		getMaxBin(bin, k, std::less<BinType>());
		x = k.first;
		y = k.second;
//		std::cout << "Histogram2D.getMaxBin(bin, x, y)" << std::endl;
	}
	void getMaxBin(BinType& bin, Keys& k) const { getMaxBin(bin, k, std::less<BinType>()); std::cout << "Histogram2D.getMaxBin(bin, k)" << std::endl;}
	template<class Comp>
	void getMaxBin(BinType& bin, Key& x, Key& y, Comp comp) const {
		Keys k = std::make_pair(x, y);
		getMaxBin(bin, k, comp);
		x = k.first;
		y = k.second;
//		std::cout << "Histogram2D.getMaxBin(bin, x, y, comp)" << std::endl;
	}
	template<class Comp>
	void getMaxBin(BinType& bin, Keys& k, Comp comp) const;
protected:
	typedef std::vector<BinType> Bins;

	size_t getIdx(const Key& x, const Key& y) const {
		size_t xIdx, yIdx;
		if ((x < mMinX) || (x > mMaxX) || (y < mMinY) || (y < mMaxY))
			throw OutOfRange("key is out of range");
		xIdx = (x - mMinX) / mBinSize;
		yIdx = (y - mMinY) / mBinSize;
		return xIdx + mXSpan * yIdx;
//		if (x == mMaxX) xIdx = mXSpan;
//		else xIdx = (x - mMinX) / mBinSize;
//		if (y == mMaxY) yIdx = mYSpan;
//		else yIdx = (y - mMinY) / mBinSize;
//		return xIdx + mXSpan * yIdx;
	}

	size_t getIdx(const size_t& xIdx, const size_t& yIdx) const {
		if ((xIdx > mXSpan) || (yIdx > mYSpan)){
			throw OutOfRange("key is out of range");
		}
		return xIdx + mXSpan * yIdx;
	}

	Keys getKeys(const size_t& idx) const {

		size_t yIdx = floor( ((float)idx) / ((float)mXSpan) );
		size_t xIdx = idx - mXSpan * yIdx;

		Key x( mMinX + ((float)xIdx) * mBinSize );
		Key y( mMinY + ((float)yIdx) * mBinSize );

		return std::make_pair(x, y);
	}

	Key mMinX, mMaxX, mMinY, mMaxY, mBinSize;
	unsigned int mXSpan, mYSpan;
	Bins mBins;
};

template<class Key, class Value, class BinType>
template<class Comp>
void Histogram2D<Key, Value, BinType>::getMaxBin(BinType& bin, Keys& k, Comp comp) const{
	typename Bins::const_iterator it = std::max_element(mBins.begin(), mBins.end(), comp);
	size_t idx = it - mBins.begin();
//	std::cout << "index of max (2D Histogram): " << idx << std::endl;
	k = getKeys(idx);
//	std::cout << "keys of max (2D Histogram): (" << k.first << ", " << k.second << ")" << std::endl;
	bin = *it;
//	std::cout << "Histogram2D.getMaxBin(bin, k, comp)" << std::endl;
}


#endif /* HISTOGRAM2D_H_ */
