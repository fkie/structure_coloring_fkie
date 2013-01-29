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
#ifndef WEIGHTEDIDXVECTOR_H_
#define WEIGHTEDIDXVECTOR_H_

#include <algorithm>

class WeightedIdx{
public:
	WeightedIdx(float w, int idx): mWeight(w), mIdx(idx)
	{}

	float& weight(){return mWeight;}
	float weight() const {return mWeight;}

	int& index(){return mIdx;}
	int index() const {return mIdx;}
private:
	float mWeight;
	int mIdx;
};

class WeightedIdxVector{
public:
	typedef std::vector<int> IdxVector;

	WeightedIdxVector(): mWeight(0.0f)
	{}

	void operator+=(const WeightedIdx& add){
		mWeight += add.weight();
		mNodeIndices.push_back(add.index());
	}

	void operator-=(const WeightedIdx& sub){
		for(IdxVector::iterator i_it = mNodeIndices.begin(); i_it != mNodeIndices.end(); ++i_it){
			if (*i_it == sub.index()){
				std::swap(*i_it, mNodeIndices.back());
				mNodeIndices.resize(mNodeIndices.size() -1);
				mWeight -= sub.weight();
				return;
			}
		}
	}

	float& weight(){return mWeight;}
	float weight() const {return mWeight;}

	IdxVector& nodeIndices(){return mNodeIndices;}
	const IdxVector& nodeIndices() const {return mNodeIndices;}
private:
	float mWeight;
	IdxVector mNodeIndices;
};

class CompareWeightedIdxVector{
public:
	bool operator()(WeightedIdxVector wiv1, WeightedIdxVector wiv2){
		return wiv1.weight() < wiv2.weight();
	}
};

#endif /* WEIGHTEDIDXVECTOR_H_ */
