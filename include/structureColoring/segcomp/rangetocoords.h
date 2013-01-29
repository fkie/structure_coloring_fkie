/*
 * rangetocoords.h
 *
 *  cpp-port for seg comp c-sourcecode
 *  see ftp://figment.csee.usf.edu/pub/segmentation-comparison/range-to-coords.c
 */

#ifndef RANGETOCOORDS_H_
#define RANGETOCOORDS_H_

void ConvertPerceptronRangeToCartesian(short int* RangeImage, double *P[3], int ROWS, int COLS);
void ConvertABWRangeToCartesian(const unsigned char* RangeImage, const int width, const int height, double* P[3], const int cal);
void ConvertIMakeRangeToCartesian(unsigned char* RangeImage, double* P[3]);

#endif /* RANGETOCOORDS_H_ */
