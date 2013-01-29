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

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "rangetocoords.h"
#include "../structures/PlanePatch.h"
#include <string>
#include <arpa/inet.h>

#ifndef RANGEIMAGEIO_H_
#define RANGEIMAGEIO_H_
#define RAS_MAGIC 0x59a66a95

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int getInt(FILE* file);

template<typename DataType>
void readPGM(FILE* fp, DataType* buffer, unsigned int& col, unsigned int& row) {
	unsigned int i, j, k = 0;
	int z;

	char filetype[3];
	int result = fscanf(fp, "%s", filetype);
	if ((result == EOF) || (result == 0))
	{
//		ROS_ERROR("Could not read first line. File not in PGM format.");
		throw std::runtime_error("File not in PGM format.");
	}
	if (strcmp(filetype, "P5") && strcmp(filetype, "P2"))
	{
		col = 0;
		row = 0;
//		ROS_ERROR("File not in PGM format");
		throw std::runtime_error("File not in PGM format.");
	}

	col = getInt(fp);
	row = getInt(fp);

	// skip comments in header *
	while (255 != (i = getInt(fp)))
		;

	if (0 == strcmp(filetype, "P5")) {
		// binary
//		for (i = 1; i <= row; i++)
//			for (j = 1; j <= col; j++)
//				buffer[k++] = (int) fgetc(fp);
		if (fread(buffer,sizeof(DataType),row * col,fp) != row * col){
//			ROS_ERROR("rasterfile error, could not read %d rows and %d cols", row, col);
			exit(1);
		}
	} else {
		// ascii
		for (i = 1; i <= row; i++)
			for (j = 1; j <= col; j++) {
				if (fscanf(fp, "%d", &z) != 1)
				{
//					ROS_ERROR("could not read value");
					throw std::runtime_error("Could not read value. File not in PGM format.");
				}
				buffer[k++] = (DataType)z;
			}
	}
}

void writePGM(const std::string& filename, const unsigned int& width, const unsigned int& height, const std::vector<unsigned char>& data);

//void readRasterfile(FILE* fp, unsigned char* buffer, unsigned int& col, unsigned int& row);

template<typename DataType>
void readRasterfile(FILE* fp, DataType * buffer, unsigned int& cols, unsigned int& rows){
	unsigned int rasterHeader[8];
	if (fread(rasterHeader, sizeof(unsigned int), 8, fp) != 8){
//		ROS_ERROR("rasterfile header could not be read!");
		exit(1);
	}
	if (rasterHeader[0] != RAS_MAGIC) {
		if (ntohl(rasterHeader[0]) != RAS_MAGIC){
//			ROS_ERROR("rasterfile magic number %u does not match %u", rasterHeader[0], RAS_MAGIC);
			exit(1);
		}
		for (unsigned int i=0; i < 8; ++i){
			rasterHeader[i] = ntohl(rasterHeader[i]);
		}
	}
	cols = rasterHeader[1];
	rows = rasterHeader[2];
	if (fread(buffer,sizeof(DataType),rows * cols,fp) != rows * cols){
//		ROS_ERROR("rasterfile error, could not read %d rows and %d cols", rows, cols);
		exit(1);
	}
}

void writeRasterfile(const std::string& filename, const unsigned int& width, const unsigned int& height, const std::vector<unsigned char>& data);

void writeMSNormals(const std::string& filename, const PlanePatch::PlanePatches& planes);

void writeTimeToFile(const std::string& filename, const double timeInSec);

void readPointCloudFromABWFile(PointCloud& pointCloud, const std::string& filename, unsigned int& width, unsigned int& height, std::vector<unsigned int>& undefPoints, bool pgm = false);
void readPointCloudFromPERCEPTRONFile(PointCloud& pointCloud, const std::string& filename, unsigned int& width, unsigned int& height, std::vector<unsigned int>& undefPoints);

#endif /* RANGEIMAGEIO_H_ */
