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

#ifndef CONNECTEDCOMPONENT_H_
#define CONNECTEDCOMPONENT_H_

#include <vector>

class ConnectedComponent{
public:
	typedef std::pair<unsigned int, unsigned int> Pair2ui;
	typedef std::vector<Pair2ui> Coords;

	ConnectedComponent(): mNumberOfPoints(0){}

	void addNumberOfPoints(const unsigned int& numPoints){mNumberOfPoints+=numPoints;}
	void addCell(Pair2ui coords){mCellsCoords.push_back(coords);}

	const Coords& getCells() const {return mCellsCoords;}
	unsigned int getNumPoints() const {return mNumberOfPoints;}

	void reset(){
		mCellsCoords.clear();
		mNumberOfPoints = 0;
	}

private:
	Coords mCellsCoords;
	unsigned int mNumberOfPoints;
};

#endif /* CONNECTEDCOMPONENT_H_ */
