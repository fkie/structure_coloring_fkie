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

#ifndef GRID2D_H_
#define GRID2D_H_

#include <vector>

#include <iostream>

template<typename CellType>
class Grid2D {
public:
	typedef CellType value_type;
	typedef std::vector<CellType> Cells;
	typedef std::pair<unsigned int, unsigned int> Indices;
	typedef std::pair<int, int> SignedIndices;

//constructor
	Grid2D() : mGrid(0, CellType()), mWidth(0), mHeight(0){}

	Grid2D(const unsigned int& width, const unsigned int& height, const CellType& c = CellType())
		: mGrid(width*height, c), mWidth(width), mHeight(height){
	}

//getter
	const unsigned int& getWidth() const { return mWidth; }
	const unsigned int& getHeight() const { return mHeight; }
	const Cells& getCells() const { return mGrid; }
	Cells& getCells() { return mGrid; }

	typename Cells::const_reference operator() (unsigned int  w, unsigned int h) const { return mGrid[getIndex(w, h, mWidth)]; }
	typename Cells::reference operator() (unsigned int w, unsigned int h) { return mGrid[getIndex(w, h, mWidth)]; }
	typename Cells::const_reference operator() (const Indices& indices) const { return (*this)(indices.first, indices.second); }
	typename Cells::reference operator() (const Indices& indices) { return (*this)(indices.first, indices.second); }

//calculations
    void getNeighbors(Cells& cells, const SignedIndices& indices, unsigned int neighborhoodRadius) const{
    	int wstart = std::max(int(0), std::min(int(mWidth), indices.first - int(neighborhoodRadius)));
    	int wend = std::max(int(0), std::min(int(mWidth), indices.first + int(neighborhoodRadius)));
    	int hstart = std::max(int(0), std::min(int(mHeight), indices.second - int(neighborhoodRadius)));
    	int hend = std::max(int(0), std::min(int(mHeight), indices.second+ int(neighborhoodRadius)));
    	for(int wind = wstart; wind < wend; ++wind){
    		for(int hind = hstart; hind < hend; ++hind){
    			if(wind < int(mWidth) && hind < int(mHeight)){
    				cells.push_back((*this)(wind, hind));
    			}
    		}
    	}
    }

protected:
    static unsigned int getIndex(unsigned int w, unsigned int h, unsigned int width) {
    	return h*width+w;
    }

	Cells mGrid;
	unsigned int mWidth;
	unsigned int mHeight;
};

#endif /* GRID2D_H_ */
