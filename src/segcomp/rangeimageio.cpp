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

#include <structureColoring/segcomp/rangeimageio.h>
#include <stdexcept>
#include <cstdio>

int getInt(FILE* file){
	bool isNumber = false;
	char item;
	do{
		item = fgetc(file);
	    if (item =='#') {   // comment
	      while (item != '\n' && item != EOF) item=fgetc(file);
	    }

	    if (item == EOF) return 0;
	    if (item >='0' && item <='9'){
	    	isNumber = true;
//	    	long int pos_byte = ftell(file);
//	    	fseek(file, pos_byte, SEEK_SET);
	    	break;
	    }

	    // illegal values
	    if ( item !='\t' && item !='\r'
	        && item !='\n' && item !=',' && item !=' ') return(-1);
	} while (!isNumber);
	if (isNumber)
	{
		ungetc(item, file);
		int i=0;
		int k = fscanf(file, "%d", &i);
		if ((k == EOF) || (k == 0)) return -1;
		item = fgetc(file);
		return i;
	}
	return -1;
}

/*****************************************************************************/

/*
**	A Perceptron image contains both 12-bit range and 12-bit reflectance,
**	packed together in one file, along with some other control bit info.
**	The following code excerpt illustrates how to unpack the 12-bit
**	range and convert it to Cartesian X,Y,Z.
*/

void readPerceptronFile(FILE* fp, short int* buffer, unsigned int& col, unsigned int& row){
	int *LoadingImage;
	char header[5][40];

	row = 512;
	col = 512;
    LoadingImage=(int *)calloc(col * row,sizeof(int));
    if (!fscanf(fp,"%s %s",header[0],header[1])) exit(1);
    if (!fscanf(fp,"%s %s",header[0],header[1])) exit(1);
    if (!fscanf(fp,"%s %s %s",header[0],header[1],header[2])) exit(1);
    if (!fscanf(fp,"%s %s %s",header[0],header[1],header[2])) exit(1);
    if (!fscanf(fp,"%s %s %s",header[0],header[1],header[2])) exit(1);
    if (!fscanf(fp,"%s %s %s",header[0],header[1],header[2])) exit(1);
    if (!fscanf(fp,"%s %s %s %s",header[0],header[1],header[2],header[3])) exit(1);
    if (!fscanf(fp,"%s %s %s %s",header[0],header[1],header[2],header[3])) exit(1);
    if (!fscanf(fp,"%s %s %s",header[0],header[1],header[2])) exit(1);
    if (!fscanf(fp,"%s %s %s %s %s",header[0],header[1],header[2],
        header[3],header[4])) exit(1);
    char crap[2];
    if(!fscanf(fp,"%c%c",crap,crap+1)) exit(1); /* extra new-line bytes */
    if(!fread(LoadingImage,sizeof(int),col * row,fp)) exit(1);
    for (unsigned int i=0; i < col * row; i++)
    {
      buffer[i]=(short int)(htonl(LoadingImage[i]) & 4095);
    }
}

/*****************************************************************************/

void writePGM(const std::string& filename, const unsigned int& width, const unsigned int& height, const std::vector<unsigned char>& data){
	FILE* file = NULL;
	if( (file = fopen(filename.c_str(), "w")) == NULL){printf("cannot open pgm file for writing"); return;}
	fprintf(file, "P5\n");
	fprintf(file, "%d %d\n", width, height);
	fprintf(file, "255\n");
	for(unsigned int i=0; i<width*height; i++){
		assert(data[i] < 255);
		fputc(data[i], file);
	}
	fclose(file);
}

/*****************************************************************************/

void writeRasterfile(const std::string& filename, const unsigned int& width, const unsigned int& height, const std::vector<unsigned char>& data){
	FILE* file = NULL;
	if( (file = fopen(filename.c_str(), "w")) == NULL){printf("cannot open rasterfile file for writing"); return;}
	int rasterHeader[8];
	rasterHeader[0] = RAS_MAGIC;
	rasterHeader[1] = width;
	rasterHeader[2] = height;
	rasterHeader[3] = 8;
	rasterHeader[4] = width * height;
	rasterHeader[5] = 1;
	rasterHeader[6] = 0;
	rasterHeader[7] = 0;
	for (unsigned int i=0; i < 8; ++i){
		rasterHeader[i] = ntohl(rasterHeader[i]);
	}
//	ROS_INFO("ras_magic = %u", rasterHeader[0]);
//	ROS_INFO("ras_width = %u", rasterHeader[1]);
//	ROS_INFO("ras_height = %u", rasterHeader[2]);
//	ROS_INFO("ras_depth = %u", rasterHeader[3]);
//	ROS_INFO("ras_length = %u", rasterHeader[4]);
//	ROS_INFO("ras_type = %u", rasterHeader[5]);
//	ROS_INFO("ras_maptype = %u", rasterHeader[6]);
//	ROS_INFO("ras_maplength = %u", rasterHeader[7]);
	fwrite(rasterHeader, sizeof(int), 8, file);
	for(unsigned int i=0; i<width*height; i++){
		fputc(data[i], file);
	}
	fclose(file);
}

/*****************************************************************************/

void writeMSNormals(const std::string& filename, const PlanePatch::PlanePatches& planes){
	FILE* file = NULL;
	if ( (file = fopen(filename.c_str(), "w")) == NULL){printf("cannot open normals file for writing"); return;}
	fprintf(file, "%d\n", (int)planes.size());
	for(PlanePatch::PlanePatches::const_iterator planes_it = planes.begin(); planes_it != planes.end(); ++planes_it){
		fprintf(file, "%d ", (int)(*planes_it)->getId());
		const Eigen::Vector3f& planeNormal((*planes_it)->getPlane3D().getPlaneNormal());
		fprintf(file, "%lf %lf %lf\n", planeNormal.x(), planeNormal.y(), planeNormal.z());
	}
	fclose(file);
}

/*****************************************************************************/

void writeTimeToFile(const std::string& filename, const double timeInSec){
	FILE* fp;
	if((fp = fopen(filename.c_str(), "a"))==NULL) return;
	fprintf(fp, "%g\n", timeInSec);
	fclose(fp);
}

/*****************************************************************************/

void readPointCloudFromABWFile(PointCloud& pointCloud, const std::string& filename, unsigned int& width, unsigned int& height, std::vector<unsigned int>& undefPoints, bool pgm){
	FILE * file = NULL;
	if ( (file = fopen(filename.c_str(),"r")) == NULL) {printf("cannot read file: %s", filename.c_str()); exit (2);}
	// obtain file size:
	fseek (file , 0 , SEEK_END);
	long int lSize = ftell (file);
	rewind(file);

	// allocate memory to contain the whole file:
	unsigned char * buffer = (unsigned char*) malloc (sizeof(unsigned char)*lSize);
	if (buffer == NULL) {printf("Memory error"); exit (5);}
	if (pgm)
		readPGM(file, buffer, width, height);
	else
		readRasterfile<unsigned char>(file, buffer, width, height);
	fclose (file);
	size_t numPoints = width*height;
//	pointCloud.width = width; does not work due to undefined points
//	pointCloud.height = height; //TODO make this work somehow
//	ROS_INFO("width %d, height %d, numPoints %lu, lSize %ld", width, height, numPoints, lSize);

	// the whole file is now loaded in the memory buffer.
	double* P[3];
	P[0] = new double[numPoints];
	P[1] = new double[numPoints];
	P[2] = new double[numPoints];
	int cal = 0;
	std::string calList[] = {"0", "19", "21", "25", "27", "28", "29"};
	unsigned int calListSize = 7;
	std::string prefix = "abw.test.";
	std::string suffix = ".range";
	for (unsigned int i = 0; i < calListSize; ++i){
		std::string fnCheck = prefix;
		fnCheck.append(calList[i]);
		fnCheck.append(suffix);
//		std::cout << fnCheck << " " << filename << std::endl;
		if (filename.find(fnCheck) != filename.npos){
			cal = 1;
//			std::cout << "chose first calibration for this file" << std::endl;
			break;
		}
	}
	ConvertABWRangeToCartesian(buffer, width, height, P, cal);
	undefPoints.clear();
	for(size_t i = 0; i < numPoints; i++){
		if ((P[0][i]!= -1.0)&&(P[1][i]!= -1.0)&&(P[2][i]!= -1.0)){
			PointT p;
			p.x = -((P[2][i] / 100.0) - 4.0);
			p.y = -P[0][i] / 100.0;
			p.z = P[1][i] / 100.0;
			p.rgb = 0x00000000;
			if (fabsf(p.x < 10) && fabsf(p.y < 10) && fabsf(p.z < 10))
				pointCloud.points.push_back(p);
			else {
				printf("point (%f, %f, %f) out of range", p.x, p.y, p.z);
				undefPoints.push_back((unsigned int)i);
			}
		} else {
			undefPoints.push_back((unsigned int)i);
		}
	}

	// terminate
	free (buffer);
	delete [](P[0]);
	delete [](P[1]);
	delete [](P[2]);
}

void readPointCloudFromPERCEPTRONFile(PointCloud& pointCloud, const std::string& filename, unsigned int& width, unsigned int& height, std::vector<unsigned int>& undefPoints){
	FILE * file = NULL;
	if ( (file = fopen(filename.c_str(),"r")) == NULL) {printf("cannot read file"); exit (2);}
	// obtain file size:
	fseek (file , 0 , SEEK_END);
	long int lSize = ftell (file);
	rewind(file);

	// allocate memory to contain the whole file:
	short int * buffer = (short int*) malloc (sizeof(short int)*lSize);
	if (buffer == NULL) {printf("Memory error"); exit (5);}
	readPerceptronFile(file, buffer, width, height);
	fclose (file);
	size_t numPoints = width*height;
//	pointCloud.width = width; does not work due to undefined points
//	pointCloud.height = height; //TODO make this work somehow
//	ROS_INFO("width %d, height %d, numPoints %lu, lSize %ld", width, height, numPoints, lSize);

	// the whole file is now loaded in the memory buffer.
	double* P[3];
	P[0] = new double[numPoints];
	P[1] = new double[numPoints];
	P[2] = new double[numPoints];

	ConvertPerceptronRangeToCartesian(buffer, P, width, height);
	undefPoints.clear();

	for(size_t i = 0; i < numPoints; i++){
		if ((P[0][i]!= -1.0)&&(P[1][i]!= -1.0)&&(P[2][i]!= -1.0)){
			PointT p;
			p.x = -((P[2][i] / 100.0) - 4.0);
			p.y = -P[0][i] / 100.0;
			p.z = P[1][i] / 100.0;
			p.rgb = 0x00000000;
			if (fabsf(p.x < 10) && fabsf(p.y < 10) && fabsf(p.z < 10))
				pointCloud.points.push_back(p);
			else {
				printf("point (%f, %f, %f) out of range", p.x, p.y, p.z);
				undefPoints.push_back((unsigned int)i);
			}
		} else {
			undefPoints.push_back((unsigned int)i);
		}
	}

	// terminate
	free (buffer);
	delete [](P[0]);
	delete [](P[1]);
	delete [](P[2]);
}
