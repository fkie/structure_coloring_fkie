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

#ifndef ROTATIONHELPER_H_
#define ROTATIONHELPER_H_

#include <Eigen/Dense>
#include <limits.h>
#include <cmath>

inline void calculateRotation(Eigen::Vector3f& axis, float& theta, const Eigen::Vector3f& a, const Eigen::Vector3f& b){
	float x = (a.dot(b)) / (std::sqrt(a.squaredNorm() * b.squaredNorm())); // with normalization
	if ((x - 1.0) > -std::numeric_limits<float>::epsilon()) { // a colinear to b in the same direction (cross(a,b) = 0)
		theta = 0.0;
		Eigen::Vector3f c(0.0, 0.0, 0.0);
		if ((a[1] * a[1] + a[2] * a[2]) < std::numeric_limits<float>::epsilon()) // a colinear to x-axis
			c[1] = 1; // y-axis
		else
			c[0] = 1; // x-axis
		axis = a.cross(c);
		axis.normalize();
	} else if ((x + 1.0) < std::numeric_limits<float>::epsilon()) { // a colinear to b in opposing directions (cross(a,b) = 0)
		theta = M_PI;
		Eigen::Vector3f c(0.0, 0.0, 0.0);
		if ((a[1] * a[1] + a[2] * a[2]) < std::numeric_limits<float>::epsilon()) // a colinear to x-axis
			c[1] = 1; // y-axis
		else
			c[0] = 1; // x-axis
		axis = a.cross(c);
		axis.normalize();
	} else {
		theta = (x < 0.0) ? M_PI - 2 * asin(0.5 * (-a - b).norm()) : 2 * asin(0.5 * (a - b).norm());
		axis = a.cross(b);
		axis.normalize();
	}
}

#endif /* ROTATIONHELPER_H_ */
