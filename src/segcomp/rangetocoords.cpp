/*
 * rangetocoords.cpp
 *
 *  cpp-port for seg comp c-sourcecode
 *  see ftp://figment.csee.usf.edu/pub/segmentation-comparison/range-to-coords.c
 */

#include <structureColoring/segcomp/rangetocoords.h>
#include <stdio.h>
#include <math.h>

#define UNDEFINED_PT	-1.0



/*
**      Convert Perceptron image to 3D cartesian coordinates.
**
**
**                                   beta > 0
**
**                                       ^ y
**                      alpha > 0        |        alpha < 0
** Seen from camera:              x <----|----                (z-axis points
**                                                             out of camera)
**
**                                   beta < 0
*/
//short int	*RangeImage;
//double		*P[3];
//int		ROWS,COLS;

void ConvertPerceptronRangeToCartesian(short int* RangeImage, double *P[3], int ROWS, int COLS)
{
double		ra, r_0, r_1, r_2, r_3, h_1, h_2, dx, dy, dz;
double		alpha, beta, alpha_0, beta_0;
			/* alpha = Horizontal, beta = Vertical angle */
double		gamma, theta;
double		H,V,delta;
int		r,c;

h_1=3.0;			/* dist (y) between rotating mirror axis
				** and the parallel laser beam. */
h_2=5.5; 			/* dist (y) between nodding mirror axis
				** and rotating mirror laser intersection. */
gamma=45.0*(M_PI/180.0);	/* slope of rotating mirror */
theta=45.0*(M_PI/180.0);	/* slope of nodding mirror in mid position */
alpha_0=beta_0=0.0;
H=51.65;
V=36.73;
r_0=830.3;
delta=0.20236;

for (r=0; r<ROWS; r++)
  {
  for (c=0; c<COLS; c++)
    {
    alpha = alpha_0 + H*(255.5 - c)/512.0;
    alpha = alpha*M_PI/180.0;
    beta  = beta_0  + V*(255.5 - r)/512.0;
    beta  = beta*M_PI/180.0;

	/* (dx,dy,dz): where the laser leaves the nodding mirror */

    dz = -(h_1*(1.0-cos(alpha))/tan(gamma));/* Motion of the laser intersection
				               point with the rotating mirror
				               as a function of alpha. */

    dy = dz*tan(theta + 0.5*beta);  /* Vertical offset of nodding mirror
				       intersection due to dz */

    dx = (h_2 + dy)*tan(alpha);     /* Horizontal offset of laser intersection
				       point with nodding mirror due to alpha */

	/*
	** R  : true range: laser light path length (laser travel) from
	**      laser emission point to target.
	** ra : range value returned by camera.
	** r_0: (standoff distance) length of laser ray to point for which r=0
	** r_1: laser travel (z) from laserdiode to intersection with rotating
	**      mirror: r1 = d1 + dz
	** r_2: laser travel (vector length) from rotating mirror
	**	to nodding mirror.
	** r_3: laser travel (vector length) from nodding mirror to target.
	**
        ** Thus: R = r + r0 = r1 + r2 + r3
	*/

    ra  = (double) RangeImage[r*COLS+c]; /* range returned by camera */

    r_1 = (dz - h_2)/delta;
    r_2 = sqrt(dx*dx + (h_2+dy)*(h_2+dy))/delta;
    r_3 = (ra + r_0 - (r_1 +r_2))*delta;

    P[0][r*COLS+c] = dx + r_3 * sin(alpha);
    P[1][r*COLS+c] = dy + r_3 * cos(alpha)*sin(beta);
    P[2][r*COLS+c] = dz + r_3 * cos(alpha)*cos(beta);
    }
  }
}

/*
 *	This routine converts an ABW range image to 3D cartesian coordinates.
 */
void ConvertABWRangeToCartesian(const unsigned char* RangeImage, const int width, const int height, double* P[3], const int cal)
{
	int r, c;
	double OFFSET;
	double SCAL;
	double F;
	double C;

	if (cal == 1) {
		OFFSET = 785.410786;
		SCAL = 0.773545;
		F = -1610.981722;
		C = 1.4508;
	} else {
		OFFSET = 771.016866;
		SCAL = 0.791686;
		F = -1586.072821;
		C = 1.4508;
	}

	for (r = 0; r < height; r++) {
		for (c = 0; c < width; c++) {
			size_t idx = r * width + c;
			if (RangeImage[idx] == 0) {
				P[0][idx] = P[1][idx] = P[2][idx]
						=UNDEFINED_PT;
				continue;
			}
			P[0][idx] = (double) (c - 255) * ((double) RangeImage[idx] / SCAL + OFFSET) / fabs(F);
			P[1][idx] = (double) (-r + 255) / C * ((double) RangeImage[idx] / SCAL + OFFSET)
					/ fabs(F);
			P[2][idx] = (double) (255 - RangeImage[idx]) / SCAL;
		}
	}
}

/*
 *	This routine converts a synthetic image created by `image-make'
 *	(used for the creating synthetic range images for the evaluation)
 *	into 3D cartesian coordinates.
 */
void ConvertIMakeRangeToCartesian(unsigned char* RangeImage, double* P[3])
{
int		r,c;
double		HORIZONTAL_FOV=60.0;
double		VERTICAL_FOV=60.0;
int		IMAGE_ROWS=512;
int		IMAGE_COLS=512;
double		RayX,RayY;

for (r=0; r<512; r++)
  {
  for (c=0; c<512; c++)
    {
    if (RangeImage[r*512+c] == 0)
      {
      P[0][r*512+c]=P[1][r*512+c]=P[2][r*512+c]=UNDEFINED_PT;
      continue;
      }
        /* Compute angles (orthogonal-axis coordinate system) for pixel,
        ** located at the _center_ of the pixel */
    RayX=((-HORIZONTAL_FOV/2.0)+((double)r+0.5)*
                (HORIZONTAL_FOV/(double)IMAGE_ROWS))*M_PI/180.0;
    RayY=((-VERTICAL_FOV/2.0)+((double)c+0.5)*
                (VERTICAL_FOV/(double)IMAGE_COLS))*M_PI/180.0;
    P[2][r*512+c]=(double)RangeImage[r*512+c]/sqrt(1.0+
	(tan(RayY)*tan(RayY))+(tan(RayX)*tan(RayX)));
    P[1][r*512+c]=P[2][r*512+c]*tan(RayY);
    P[0][r*512+c]=P[2][r*512+c]*tan(RayX);
    }
  }
}
