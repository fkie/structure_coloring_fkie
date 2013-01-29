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

#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include <functional>
#include <algorithm>
#include <vector>
#include <structureColoring/histograms/OutOfRangeException.h>

/** \brief Histogram class.
 * \tparam Key usually some number type. Key must support operators <,>,==,+,-,* and /.
 * \tparam Value the value type, which can be added to bins.
 * \tparam BinType the bin type. BinType must have operator+=(const Value& v).
 */
template<class Key, class Value, class BinType = Value>
class Histogram 
{
public:
	typedef OutOfRangeException OutOfRange;

	/** \brief Constructor.
	 * \param min minimum key value (inclusive).
	 * \param max maximum key value (inclusive). The key max itself is allocated to the last bin.
	 * \param binSize the size of one bin.
	 */
	Histogram(const Key& min, const Key& max, const Key& binSize)
		: mMin(min), mMax(max), mBinSize(binSize), mBins(static_cast<size_t>((mMax - mMin)/mBinSize))
	{}
	BinType& bin(const Key& k) { return mBins[getIdx(k)];}
	const BinType& bin(const Key& k) const { return mBins[getIdx(k)];}
	void addToBin(const Key& k, const Value& v) { mBins[getIdx(k)] += v;}
	void getMaxBin(BinType& bin, Key& k) const { getMaxBin(bin, k, std::less<BinType>());}
	template<class Comp>
	void getMaxBin(BinType& bin, Key& k, Comp comp) const;
protected:
	typedef std::vector<BinType> Bins;
	size_t getIdx(const Key& k) const;
	Key getKey(const size_t& idx) const;
	Key mMin, mMax, mBinSize;
	Bins mBins;
};

template<class Key, class Value, class BinType>
size_t Histogram<Key, Value, BinType>::getIdx(const Key& k) const
{
	if ((k < mMin) || (k > mMax))
		throw OutOfRange("key is out of range");
	if (k == mMax)
		return mBins.size()-1; //construction of mBins is to short to hold mMax!
	return static_cast<size_t>((k - mMin)/mBinSize);
}

template<class Key, class Value, class BinType>
Key Histogram<Key, Value, BinType>::getKey(const size_t& idx) const
{
	return (Key(idx) * mBinSize) + mMin;
}

template<class Key, class Value, class BinType>
template<class Comp>
void Histogram<Key, Value, BinType>::getMaxBin(BinType& bin, Key& k, Comp comp) const
{
	typename Bins::const_iterator it = std::max_element(mBins.begin(), mBins.end(), comp);
	size_t idx = it - mBins.begin();
	k = getKey(idx);
	bin = *it;
//	std::cout << "Histogram.getMaxBin(bin, k, comp) called: number of bins: " << mBins.size() << "; mMin = " << mMin << "; mMax = " << mMax << std::endl;
//	std::cout << "Indices of (min, max) = (" << getIdx(mMin) << ", " << getIdx(mMax) << ")" << std::endl;
}

#endif
