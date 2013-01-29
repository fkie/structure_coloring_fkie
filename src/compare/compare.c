
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define		RAS_MAGIC	0x59a66a95	/* rasterfile magic number */

#define		MAX_REGIONS	246	/* max number of surface regions */
#define		MAX_LABELS	256	/* restricted by data structure types */
#define		MAX_ANGLES	100	/* max number of angles in GT */
#define		FW_LABELS	19	/* field widths, in characters, used */
#define		FW_MEASURES	12	/* in output file, for labels columns,*/
#define		FW_CLASSIFS	14	/* measures columns and classif.s col */
#define		BLANKS		"                         "


/******************** IMPORTANT -- EDIT PATHS ********************/
/* paths to search for GT and MS files */
char		*PATHS[]={"./",	/* leave me first! */
			NULL /* must be at end! */};

/* NOISE_DEFINED => which of the 10 labels are used */
int		NOISE_DEFINED[10] =   {1,1,1,1,0,0,0,0,0,0  };
/* globals for ROWS and COLS */
int		ROWS,COLS;
/* global for compare threshold T */
double		T_SEG;

int		LoadFiles();
void		ComputeTables();
void		ClassifyRegions();
double		AngleBetweenVectors();
void		OutputTable();
void		OutputImage();

/*
 *  This program will take as input two segmented images, one segmented 
 *  by a machine algorithm (MS), the other by a hand segmentation tool (GT).
 *  Treating the hand segmentation as the correct result, this tool  
 *  compares them to evaluate the performance of the machine routine.   
 */

int main(argc,argv)

int	argc;
char	*argv[];

{
unsigned char	*GTImage,*MSImage;
char		FilePrefix[160],Filename[150],text[500];
int		i,j,k;
char		GTRegionClassifications[MAX_LABELS];
char		MSRegionClassifications[MAX_LABELS];
int		GTRegionMappings[MAX_LABELS],MSRegionMappings[MAX_LABELS];
double		GTMappingMeasures[MAX_LABELS][2];
double		MSMappingMeasures[MAX_LABELS][2];
int		OverlapTable[MAX_LABELS][MAX_LABELS];
int		GTRegionSizes[MAX_LABELS],MSRegionSizes[MAX_LABELS];
int		TotalRegionsInMS,TotalRegionsInGT;
int		TotalCorrectClassifications,TotalOversegClassifications;
int		TotalUndersegClassifications,TotalAnglesInGT;
int		TotalMissedClassifications,TotalNoiseClassifications;
int		TotalCorrectPixelCount, TotalGTPixelCount, TotalNotCorrectMSPixels;
int		GTAngleRegions[MAX_ANGLES][2];
double		GTAngleValues[MAX_ANGLES];
double		MSRegionNormals[MAX_LABELS][3];
int		TotalAnglesCompared;
double		C_Avg_m1,C_Avg_m2,O_Avg_m1,O_Avg_m2,U_Avg_m1,U_Avg_m2;
double		AvgAngDiff,stddev;
int		PerImageGTRegions,PerImageCorrect;
int		PerImageOverseg,PerImageUnderseg,PerImageMissed;
int		PerImageNoise,TotalImagesCompared;
int		PerImageCorrectPixels, PerImageGTPixels, PerImageNotCorrectMSPixels;
double		PerImageAngDiff,PerImageStdDev,TotalImagesWithAnglesCompared;
int		PrintFile,OutputImages,StartImage,EndImage,PrintHelp;
int             NoNormals;
int		Sequence[50],SequenceTotal;



OutputImages=StartImage=EndImage=SequenceTotal=PrintHelp=0;
NoNormals=0;
T_SEG=0.51;
if (argc > 1)
  {
  for (i=1; i<argc; i++)
    {
    if (strcmp(argv[i],"-images") == 0)
      OutputImages=1;
    else if (strcmp(argv[i],"-start") == 0)
      {
      if (i == argc-1  ||  SequenceTotal != 0)
	break;
      StartImage=atoi(argv[i+1]);
      i++;
      }
    else if (strcmp(argv[i],"-end") == 0)
      {
      if (i == argc-1  ||  SequenceTotal != 0)
	break;
      EndImage=atoi(argv[i+1]);
      i++;
      }
    else if (strcmp(argv[i],"-sequence") == 0)
      {
      if (i == argc-1  ||  StartImage != 0  ||  EndImage != 0)
	break;
      i++;
      j=0;
      do
        {
	k=0;
	do
	  {
	  text[k]=argv[i][j+k];
	  k++;
	  }
	while ((int)strlen(argv[i]) > j+k  &&  argv[i][j+k] != ',');
	text[k]='\0';
	j+=(k+1);
	Sequence[SequenceTotal]=atoi(text);
	SequenceTotal++;
        }
      while (strlen(argv[i]) > j);
      EndImage=SequenceTotal-1;
      }
    else if (strcmp(argv[i],"-T") == 0)
      {
      if (i == argc-1)
	break;
      T_SEG=atof(argv[i+1]);
      if (T_SEG <= 0.5  ||  T_SEG > 1.0)
	PrintHelp=1;
      i++;
      }
    else if (strcmp(argv[i],"-noNormals") == 0)
      {
        NoNormals = 1;
      }
    else if (i > 1)
      break;
    }
  if (i != argc)
    PrintHelp=1;
  if (StartImage > EndImage)
    StartImage=EndImage;
  }

if (argc == 1  ||  PrintHelp == 1)
  {
  printf("\nUsage:  compare [prefix] [-start #] [-end #]\n");
  printf("                [-sequence #[,#...]] [-images] [-T #]\n\n");
  printf("This program will look for the following images:\n");
  printf("\t[prefix].gt-seg, [prefix].gt-ang  (ground truth files)\n");
  printf("\t[prefix].ms-seg, [prefix].ms-nor  (machine segmentation files)\n");
  printf("...in the following directories:\n");
  i=0;
  while (PATHS[i])
    printf("\t%s\n",PATHS[i++]);
  printf("and will write the file ./[prefix].compared (table file)\n");
  printf("If -images is specified, the program will also write:\n");
  printf("\t./[prefix].gt-lab (labeled GT compare image)\n");
  printf("\t./[prefix].ms-lab (labeled MS compare image)\n");
  printf("If -start # and/or -end # is specified, the program will run on\n");
  printf("\tthe sequence of images [prefix].[start#] ... [prefix].[end#],\n");
  printf("\tcomputing averages across the image sequence.\n");
  printf("If -sequence #[,#...] is specified, the program will run on the\n");
  printf("\tarbitrary sequence of image numbers given and report averages.\n");
  printf("The compare threshold can be set using -T.  Valid values\n");
  printf("\tare 0.5 < T <= 1.0; the default is 0.51.\n");
  printf("If no normal Information should be used, start the program with\n");
  printf("\t -noNormals\n\n");
  if (PrintHelp == 1)
    exit(1);
  printf("Enter range image filename prefix: ");
  if(!scanf("%s",Filename))return(1);
  }
else
  strcpy(Filename,argv[1]);

PerImageGTRegions=PerImageCorrect=PerImageAngDiff=PerImageStdDev=0;
PerImageOverseg=PerImageUnderseg=PerImageMissed=PerImageNoise=0;
PerImageCorrectPixels=PerImageGTPixels=PerImageNotCorrectMSPixels=0;
TotalImagesCompared=TotalImagesWithAnglesCompared=0;
TotalCorrectPixelCount=TotalGTPixelCount=TotalNotCorrectMSPixels=0;
for (j=StartImage; j<=EndImage; j++)
  {
  if (StartImage == EndImage)
    strcpy(FilePrefix,Filename);
  else if (SequenceTotal != 0)
    sprintf(FilePrefix,"%s.%d",Filename,Sequence[j]);
  else
    sprintf(FilePrefix,"%s.%d",Filename,j);

  printf("Loading ground truth files...\n");
  fflush(stdout);
  if (LoadFiles(FilePrefix,&ROWS,&COLS,&GTImage,&TotalRegionsInGT,
	&TotalAnglesInGT,GTAngleRegions,GTAngleValues,
	&MSImage,&TotalRegionsInMS,MSRegionNormals,&NoNormals) != 1)
    continue;


  printf("Computing tables...\n");
  fflush(stdout);
	/* Compute the 3 tables needed for region classification */
  ComputeTables(GTImage,MSImage,OverlapTable,GTRegionSizes,MSRegionSizes,
		TotalRegionsInGT,TotalRegionsInMS);


  for (i=0; i<MAX_LABELS; i++)
    {
    GTRegionClassifications[i]=MSRegionClassifications[i]=' ';
    GTRegionMappings[i]=MSRegionMappings[i]=0;
    GTMappingMeasures[i][0]=GTMappingMeasures[i][1]=0.0;
    MSMappingMeasures[i][0]=MSMappingMeasures[i][1]=0.0;
    }
  printf("Performing classifications...\n");
  fflush(stdout);
  ClassifyRegions(OverlapTable,GTRegionSizes,MSRegionSizes,
	TotalRegionsInGT,TotalRegionsInMS,
	GTRegionClassifications,GTRegionMappings,GTMappingMeasures,
	MSRegionClassifications,MSRegionMappings,MSMappingMeasures,
	&TotalCorrectPixelCount, &TotalGTPixelCount, &TotalNotCorrectMSPixels);



  if (StartImage != EndImage)
    PrintFile=0;	/* to compute averages without writing file */
  else
    PrintFile=1;
  OutputTable(FilePrefix,TotalRegionsInGT,TotalRegionsInMS,
	TotalAnglesInGT,GTAngleRegions,GTAngleValues,MSRegionNormals,
	GTRegionClassifications,GTRegionMappings,GTMappingMeasures,
	MSRegionClassifications,MSRegionMappings,MSMappingMeasures,
	PrintFile,
	&TotalCorrectClassifications,&C_Avg_m1,&C_Avg_m2,
	&TotalAnglesCompared,&AvgAngDiff,&stddev,
	&TotalOversegClassifications,&O_Avg_m1,&O_Avg_m2,
	&TotalUndersegClassifications,&U_Avg_m1,&U_Avg_m2,
	&TotalMissedClassifications,&TotalNoiseClassifications, &NoNormals);
  printf("**** %s:\n",FilePrefix);
  printf("\t  GT reg: %d  Correct: %d",
	TotalRegionsInGT,TotalCorrectClassifications);
  if(NoNormals)printf("\n");
  else printf("   Angle:  %f   StdDev:  %f\n",
	AvgAngDiff,stddev);
  printf("\t  Overseg: %d  Underseg: %d  Missed: %d  Noise: %d\n",
	TotalOversegClassifications,TotalUndersegClassifications,
	TotalMissedClassifications,TotalNoiseClassifications);
  printf("\t  #pixel in correct regions: %d  Total: %d  InPercent: %lf\n",
		  TotalCorrectPixelCount, TotalGTPixelCount,
		  ((double)TotalCorrectPixelCount/(double)TotalGTPixelCount)*100);
  printf("\t  #pixel missed in gt: %d  #pixel not correct in ms: %d\n",
		  TotalGTPixelCount - TotalCorrectPixelCount, TotalNotCorrectMSPixels);
  PerImageGTRegions+=TotalRegionsInGT;
  PerImageCorrect+=TotalCorrectClassifications;
  PerImageAngDiff+=AvgAngDiff;
  PerImageStdDev+=stddev;
  PerImageOverseg+=TotalOversegClassifications;
  PerImageUnderseg+=TotalUndersegClassifications;
  PerImageMissed+=TotalMissedClassifications;
  PerImageNoise+=TotalNoiseClassifications;
  PerImageCorrectPixels+=TotalCorrectPixelCount;
  PerImageGTPixels+=TotalGTPixelCount;
  PerImageNotCorrectMSPixels+=TotalNotCorrectMSPixels;
  TotalImagesCompared++;
  if (TotalAnglesCompared > 0)
    TotalImagesWithAnglesCompared++;

	/* write out labeled image files, if requested */
  if (OutputImages == 1)
    {
    printf("Writing MS labeled compare image...\n");
    fflush(stdout);
    sprintf(text,"%s.ms-lab",FilePrefix);
    OutputImage(text,MSImage,TotalRegionsInMS,MSRegionClassifications);
    printf("Writing GT labeled compare image...\n");
    fflush(stdout);
    sprintf(text,"%s.gt-lab",FilePrefix);
    OutputImage(text,GTImage,TotalRegionsInGT,GTRegionClassifications);
    }
  }

	/* print averages across image sequence (if it was a sequence) */
if (StartImage != EndImage)
  {
  if (SequenceTotal == 0)
    printf("\n**** AVERAGES %s.%d ... %s.%d\n",Filename,StartImage,
	Filename,EndImage);
  else
    {
    printf("\n**** AVERAGES %s.%d",Filename,Sequence[0]);
    for (j=1; j<SequenceTotal; j++)
      printf(",%d",Sequence[j]);
    printf("\n");
    }
  printf(" GT reg: %f   Correct:  %f",
	(float)PerImageGTRegions/(float)TotalImagesCompared,
	(float)PerImageCorrect/(float)TotalImagesCompared);
  if(NoNormals)printf("\n");
  else printf("   Angle:  %f   StdDev:  %f\n",
	(float)PerImageAngDiff/(float)TotalImagesWithAnglesCompared,
	(float)PerImageStdDev/(float)TotalImagesWithAnglesCompared);
  printf(" Overseg: %f  Underseg: %f  Missed: %f  Noise: %f\n",
	(float)PerImageOverseg/(float)TotalImagesCompared,
	(float)PerImageUnderseg/(float)TotalImagesCompared,
	(float)PerImageMissed/(float)TotalImagesCompared,
	(float)PerImageNoise/(float)TotalImagesCompared);
  printf(" Correct pixels: %f  GT pixels: %f  Percent: %f\n",
	(double)PerImageCorrectPixels/(double)TotalImagesCompared,
	(double)PerImageGTPixels/(double)TotalImagesCompared,
	((double)PerImageCorrectPixels/(double)PerImageGTPixels)*100);
  printf(" Missed Pixels in GT: %f  Not Correct Pixels in MS: %f\n",
	((double)(PerImageGTPixels - PerImageCorrectPixels))/(double)TotalImagesCompared,
	(double)PerImageNotCorrectMSPixels/(double)TotalImagesCompared);
  }

}






void ComputeTables(GTImage, MSImage, OverlapTable, GTRegionSizes,
	MSRegionSizes, TotalRegionsInGT, TotalRegionsInMS)

unsigned char	*GTImage, *MSImage;
int		OverlapTable[MAX_LABELS][MAX_LABELS];
int		GTRegionSizes[MAX_LABELS],MSRegionSizes[MAX_LABELS];
int		TotalRegionsInGT,TotalRegionsInMS;

{
int	r,c,i;

	/* Initialize all table entries to 0 (pixels) */
for (r=0; r<MAX_LABELS; r++)
  {
  for (c=0; c<MAX_LABELS; c++)
    OverlapTable[r][c]=0;
  GTRegionSizes[r]=MSRegionSizes[r]=0;
  }
	/* Scan through images and simply count */
for (i=0; i<ROWS*COLS; i++)
  {
  GTRegionSizes[GTImage[i]]++;
  MSRegionSizes[MSImage[i]]++;
  OverlapTable[MSImage[i]][GTImage[i]]++;
  }
	/* Done */
}






void ClassifyRegions(OverlapTable,GTRegionSizes,MSRegionSizes,
	TotalRegionsInGT,TotalRegionsInMS,
	GTRegionClassifications,GTRegionMappings,GTMappingMeasures,
	MSRegionClassifications,MSRegionMappings,MSMappingMeasures,
	TotalCorrectPixelCount, TotalGTPixelCount, TotalNotCorrectMSPixels)

int	OverlapTable[MAX_LABELS][MAX_LABELS];	/* overlap of regions' pixels;
						** first index is MS label,
						** second index is GT label. */
int	GTRegionSizes[MAX_LABELS];		/* size of GT regions (pixels)*/
int	MSRegionSizes[MAX_LABELS];		/* size of MS regions (pixels)*/
int	TotalRegionsInGT,TotalRegionsInMS;
char	GTRegionClassifications[MAX_LABELS];	/* type of mapping for region */
int	GTRegionMappings[MAX_LABELS];		/* if MS side of mapping is
						** only 1 region, then which
						** MS region it maps to */
double	GTMappingMeasures[MAX_LABELS][2];	/* _TOTAL_ metric measure for
						** mappings, indexed by GT
						** region labels; first index
						** is GT side of mapping,
						** second is MS side. */
char	MSRegionClassifications[MAX_LABELS];	/* type of mapping for region */
int	MSRegionMappings[MAX_LABELS];		/* if GT side of mapping is
						** only 1 region, then which
						** GT region it maps to */
double	MSMappingMeasures[MAX_LABELS][2];	/* _TOTAL_ metric measure for
						** mappings, indexed by MS
						** region labels; first index
						** is GT side of mapping,
						** second is MS side. */
int		*TotalCorrectPixelCount;
int		*TotalGTPixelCount;
int		*TotalNotCorrectMSPixels;

{
int	m,n,HowManyInMapping,TotalMSArea,TotalGTArea;
int	TotalOverlap,NewMappingIsBetter;
double	Measure1,Measure2,SoloMeasure;

*TotalCorrectPixelCount=*TotalGTPixelCount=*TotalNotCorrectMSPixels=0;

	/* Scan through table and find all CORRECT_DETECTION mappings */

for (n=10; (n-10)<TotalRegionsInGT; n++)
  {
  (*TotalGTPixelCount)+=GTRegionSizes[n];
  for (m=10; (m-10)<TotalRegionsInMS; m++)
    {
    Measure1=(double)OverlapTable[m][n]/(double)GTRegionSizes[n];
    Measure2=(double)OverlapTable[m][n]/(double)MSRegionSizes[m];
    if (Measure1 >= T_SEG  &&  Measure2 >= T_SEG)
      {
      GTRegionClassifications[n]=MSRegionClassifications[m]='c';
      GTRegionMappings[n]=m;
      MSRegionMappings[m]=n;
      GTMappingMeasures[n][0]=MSMappingMeasures[m][0]=Measure1;
      GTMappingMeasures[n][1]=MSMappingMeasures[m][1]=Measure2;
      (*TotalCorrectPixelCount)+=OverlapTable[m][n];
      (*TotalNotCorrectMSPixels)+=MSRegionSizes[m]-OverlapTable[m][n];
      }
    }
  }

	/* Now scan through table looking for OVER-SEGMENTATION mappings.
	** If one contradicts an already-found mapping, then take the
	** mapping with the highest measure. */

for (n=10; (n-10)<TotalRegionsInGT; n++)
  {
  HowManyInMapping=0;
  TotalOverlap=TotalMSArea=0;
  for (m=10; (m-10)<TotalRegionsInMS; m++)
    {
    SoloMeasure=(double)OverlapTable[m][n]/(double)MSRegionSizes[m];
    if (SoloMeasure >= T_SEG)
      {
      HowManyInMapping++;
      TotalMSArea+=MSRegionSizes[m];
      TotalOverlap+=OverlapTable[m][n];
      MSRegionMappings[m]=n;
      }
    }
  if (HowManyInMapping > 1)
    {
    Measure1=(double)TotalOverlap/(double)GTRegionSizes[n];
    Measure2=(double)TotalOverlap/(double)TotalMSArea;
    if (Measure1 >= T_SEG  &&  Measure2 >= T_SEG)
      {
      NewMappingIsBetter=1;
      if (GTRegionClassifications[n] != ' ')
		/* Has a classification already.  See if this one is higher. */
        if ((Measure1+Measure2)/2.0 <= (GTMappingMeasures[n][0]+
		GTMappingMeasures[n][1])/2.0)
          NewMappingIsBetter=0;
      if (NewMappingIsBetter)
        {
        GTRegionClassifications[n]='o';
        GTMappingMeasures[n][0]=Measure1;
        GTMappingMeasures[n][1]=Measure2;
        for (m=10; (m-10)<TotalRegionsInMS; m++)
          if (MSRegionMappings[m] == n)
            {
            MSRegionClassifications[m]='o';
            MSMappingMeasures[m][0]=Measure1;
            MSMappingMeasures[m][1]=Measure2;
            }
        }
      }
    }
  }

	/* Now scan through table looking for UNDER-SEGMENTATION mappings.
	** Again, if one contradicts an already-found mapping, then take the
	** mapping with the highest measure. */

for (m=10; (m-10)<TotalRegionsInMS; m++)
  {
  TotalNotCorrectMSPixels+=MSRegionSizes[m];
  HowManyInMapping=0;
  TotalOverlap=TotalGTArea=0;
  for (n=10; (n-10)<TotalRegionsInGT; n++)
    {
    SoloMeasure=(double)OverlapTable[m][n]/(double)GTRegionSizes[n];
    if (SoloMeasure >= T_SEG)
      {
      HowManyInMapping++;
      TotalGTArea+=GTRegionSizes[n];
      TotalOverlap+=OverlapTable[m][n];
      GTRegionMappings[n]=m;
      }
    }
  if (HowManyInMapping > 1)
    {
    Measure1=(double)TotalOverlap/(double)TotalGTArea;
    Measure2=(double)TotalOverlap/(double)MSRegionSizes[m];
    if (Measure1 >= T_SEG  &&  Measure2 >= T_SEG)
      {
      NewMappingIsBetter=1;
      if (MSRegionClassifications[m] != ' ')
		/* Has a classification already.  See if this one is higher. */
        if ((Measure1+Measure2)/2.0 <= (MSMappingMeasures[m][0]+
		MSMappingMeasures[m][1])/2.0)
          NewMappingIsBetter=0;
      if (NewMappingIsBetter)
        {
        MSRegionClassifications[m]='u';
        MSMappingMeasures[m][0]=Measure1;
        MSMappingMeasures[m][1]=Measure2;
        for (n=10; (n-10)<TotalRegionsInGT; n++)
          if (GTRegionMappings[n] == m)
            {
            GTRegionClassifications[n]='u';
            GTMappingMeasures[n][0]=Measure1;
            GTMappingMeasures[n][1]=Measure2;
            }
        }
      }
    }
  }

	/* Any GT regions not participating in mappings are classified
	** as MISSED. */

for (n=10; (n-10)<TotalRegionsInGT; n++)
  if (GTRegionClassifications[n] == ' ')
    GTRegionClassifications[n]='m';

	/* Any MS regions not participating in mappings are classified
	** as NOISE. */

for (m=10; (m-10)<TotalRegionsInMS; m++)
  if (MSRegionClassifications[m] == ' ')
    MSRegionClassifications[m]='n';

	/* Done */
}






double	AngleBetweenVectors(Vector1,Vector2)

double	Vector1[3],Vector2[3];

{
double	length1,length2,dotproduct,arc,Angle;

dotproduct=Vector1[0]*Vector2[0]+Vector1[1]*Vector2[1]+Vector1[2]*Vector2[2];
length1=sqrt(pow(Vector1[0],2.0)+pow(Vector1[1],2.0)+pow(Vector1[2],2.0));
length2=sqrt(pow(Vector2[0],2.0)+pow(Vector2[1],2.0)+pow(Vector2[2],2.0));
arc=dotproduct/(length1*length2);
if (fabs(arc) < 1.0)
  Angle=acos(fabs(arc));
else
if (arc >= 1.0)
  Angle=0.0;
else
  Angle=M_PI;
Angle *= (180.0/M_PI);
return(Angle);
}





void OutputTable(FilePrefix,TotalRegionsInGT,TotalRegionsInMS,
	TotalAnglesInGT,GTAngleRegions,GTAngleValues,MSRegionNormals,
	GTRegionClassifications,GTRegionMappings,GTMappingMeasures,
	MSRegionClassifications,MSRegionMappings,MSMappingMeasures,
	PrintFile,
	TotalCorrectClassifications,C_Avg_m1,C_Avg_m2,
	TotalAnglesCompared,AvgAngDiff,stddev,
	TotalOversegClassifications,O_Avg_m1,O_Avg_m2,
	TotalUndersegClassifications,U_Avg_m1,U_Avg_m2,
	TotalMissedClassifications,TotalNoiseClassifications, NoNormals)

char		FilePrefix[];
int		TotalRegionsInGT,TotalRegionsInMS,TotalAnglesInGT;
int		GTAngleRegions[MAX_ANGLES][2];
double		GTAngleValues[MAX_ANGLES];
double		MSRegionNormals[MAX_LABELS][3];
char		GTRegionClassifications[MAX_LABELS];
char		MSRegionClassifications[MAX_LABELS];
int		GTRegionMappings[MAX_LABELS],MSRegionMappings[MAX_LABELS];
double		GTMappingMeasures[MAX_LABELS][2];
double		MSMappingMeasures[MAX_LABELS][2];
int		PrintFile;
int		*TotalCorrectClassifications,*TotalOversegClassifications;
int		*TotalUndersegClassifications;
int		*TotalMissedClassifications,*TotalNoiseClassifications;
int		*TotalAnglesCompared;
double		*C_Avg_m1,*C_Avg_m2,*O_Avg_m1,*O_Avg_m2,*U_Avg_m1,*U_Avg_m2;
double		*AvgAngDiff,*stddev;
int		*NoNormals;

{
char	text[500],buffer[30],line[81],temp[30];
FILE	*fpt;
int	i,j,k,a,LabelCount;
double	SumOfAngles,AngleCompared[MAX_ANGLES],sigma;

*TotalCorrectClassifications=*TotalOversegClassifications=0;
*TotalUndersegClassifications=*TotalMissedClassifications=0;
*TotalNoiseClassifications=*TotalAnglesCompared=0;
*C_Avg_m1=*C_Avg_m2=*O_Avg_m1=*O_Avg_m2=*U_Avg_m1=*U_Avg_m2=0.0;
*AvgAngDiff=*stddev=0.0;
fpt=0;

if (PrintFile)
  {
  strcpy(text,FilePrefix);
  strcat(text,".compared");
  if ((fpt=fopen(text,"w")) == NULL)
    {
    fprintf(stderr,"Couldn't open %s for writing.\n",text);
    exit(1);
    }
  printf("Writing results file...\n");
  fflush(stdout);
	/* Header */
  sprintf(buffer,"GT Region(s)");
  strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
  strcpy(line,buffer);
  sprintf(buffer,"Category");
  strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
  strcat(line,buffer);
  sprintf(buffer,"MS Region(s)");
  strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
  strcat(line,buffer);
  sprintf(buffer,"Measure1");
  strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
  strcat(line,buffer);
  sprintf(buffer,"Measure2");
  strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
  strcat(line,buffer);
  fprintf(fpt,"%s\n",line);
  strcpy(line,"=");
  for (i=1; i<(FW_LABELS*2+FW_MEASURES*2+FW_CLASSIFS); i++)
    strcat(line,"=");
  fprintf(fpt,"%s\n",line);
  }

	/* Print out all correct detection mappings together */
*TotalAnglesCompared = *TotalCorrectClassifications = 0;
*C_Avg_m1 = *C_Avg_m2 = 0.0;
SumOfAngles=0.0;
for (i=10; (i-10)<TotalRegionsInGT; i++)
  {
  if(GTRegionClassifications[i] == 'c')
    {
    sprintf(buffer,"%d",i);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcpy(line,buffer);
    sprintf(buffer,"correct");
    strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%d",GTRegionMappings[i]);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",GTMappingMeasures[i][0]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",GTMappingMeasures[i][1]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    if (PrintFile)
      fprintf(fpt,"%s\n",line);
    (*C_Avg_m1) += GTMappingMeasures[i][0];
    (*C_Avg_m2) += GTMappingMeasures[i][1];
    (*TotalCorrectClassifications)++;
    if(!(*NoNormals))
    {
    for (a=0; a<TotalAnglesInGT; a++)
      {
      if (GTAngleRegions[a][0] == i)
        {
        if (GTRegionClassifications[GTAngleRegions[a][1]] == 'c')
          {
          if(GTAngleValues[a] > 90)
          	GTAngleValues[a]=fabs(GTAngleValues[a] - 180);
          AngleCompared[*TotalAnglesCompared]=
		fabs(GTAngleValues[a]-AngleBetweenVectors(
		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][0]]],
		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][1]]]));
	  SumOfAngles+=AngleCompared[*TotalAnglesCompared];
	  if(AngleCompared[*TotalAnglesCompared] > 15.0){
	    printf("compared angle between gtregions %d and %d is greater than 15 degrees: %lf\n",
	    	GTAngleRegions[a][0], GTAngleRegions[a][1], AngleCompared[*TotalAnglesCompared]);
	    printf("angle should be %lf but has value %lf\n", GTAngleValues[a], AngleBetweenVectors(
		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][0]]],
		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][1]]]));
	    printf("normals are: (%lf, %lf, %lf)\n", MSRegionNormals[GTRegionMappings[GTAngleRegions[a][0]]][0],
	    		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][0]]][1],
	    		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][0]]][2]);
	    printf("and: (%lf, %lf, %lf)\n", MSRegionNormals[GTRegionMappings[GTAngleRegions[a][1]]][0],
	    		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][1]]][1],
	    		MSRegionNormals[GTRegionMappings[GTAngleRegions[a][1]]][2]);
	  }
	  (*TotalAnglesCompared)++;
          }
        }
      }
    }
    }
  }
if(*TotalCorrectClassifications > 0)
  {
  (*C_Avg_m1) /= (double)(*TotalCorrectClassifications);
  (*C_Avg_m2) /= (double)(*TotalCorrectClassifications);
  }

	/* Get average and standard deviation of angular differences */
if(!(*NoNormals) && *TotalAnglesCompared > 0)
  {
  *AvgAngDiff=SumOfAngles/(double)(*TotalAnglesCompared);
  sigma=0.0;
  for(i=0; i<*TotalAnglesCompared; i++)
    sigma += pow((AngleCompared[i]-(*AvgAngDiff)),2.0);
  *stddev = sqrt(sigma/(*TotalAnglesCompared));
  }

	/* Print out all over-segmentation mappings together */
*TotalOversegClassifications = 0;
*O_Avg_m1 = *O_Avg_m2 = 0.0;
for (i=10; (i-10)<TotalRegionsInGT; i++)
  {
  if(GTRegionClassifications[i] == 'o')
    {
    sprintf(buffer,"%d",i);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcpy(line,buffer);
    sprintf(buffer,"over-seg");
    strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
    strcat(line,buffer);
    LabelCount=0;
    for (j=10; j<TotalRegionsInMS+10; j++)
      {
      if(MSRegionMappings[j] == i)
        {
        if (LabelCount == 0)
          sprintf(buffer,"%d",j);
        else
          {
  	  sprintf(temp,",%d",j);
          if (strlen(buffer)+strlen(temp) > FW_LABELS-1)
  	    {
  	    strcat(buffer,",");
  	    break;
  	    }
          strcat(buffer,temp);
          }
        LabelCount++;
        }
      }
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",GTMappingMeasures[i][0]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",GTMappingMeasures[i][1]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    if (PrintFile)
      fprintf(fpt,"%s\n",line);
    (*O_Avg_m1) += GTMappingMeasures[i][0];
    (*O_Avg_m2) += GTMappingMeasures[i][1];
    (*TotalOversegClassifications)++;
    while (j != TotalRegionsInMS+10)
      {
      LabelCount=0;
      for (k=j; k<TotalRegionsInMS+10; k++)
        {
        if(MSRegionMappings[k] == i)
	  {
	  if (LabelCount == 0)
	    sprintf(buffer,"%d",k);
	  else
            {
            sprintf(temp,",%d",k);
	    if (strlen(buffer)+strlen(temp) > FW_LABELS-1)
              break;
            strcat(buffer,temp);
            }
          LabelCount++;
	  }
        }
      strncpy(line,BLANKS,FW_LABELS);
      line[FW_LABELS]='\0';
      strncat(line,BLANKS,FW_CLASSIFS);
      strcat(line,buffer);
      if (PrintFile)
        fprintf(fpt,"%s\n",line);
      j=k;
      }
    }  
  }
if(*TotalOversegClassifications > 0)
  {
  (*O_Avg_m1) /= (double)(*TotalOversegClassifications);
  (*O_Avg_m2) /= (double)(*TotalOversegClassifications);
  }


	/* Print out all under-segmentation mappings together */
*TotalUndersegClassifications = 0;
*U_Avg_m1 = *U_Avg_m2 = 0.0;
for (i=10; (i-10)<TotalRegionsInMS; i++)
  {
  if(MSRegionClassifications[i] == 'u')
    {
    LabelCount=0;
    for (j=10; j<TotalRegionsInGT+10; j++)
      {
      if(GTRegionMappings[j] == i)
        {
        if (LabelCount == 0)
          sprintf(buffer,"%d",j);
        else
          {
  	  sprintf(temp,",%d",j);
          if (strlen(buffer)+strlen(temp) > FW_LABELS-1)
  	    {
  	    strcat(buffer,",");
  	    break;
  	    }
          strcat(buffer,temp);
          }
        LabelCount++;
        }
      }
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcpy(line,buffer);
    sprintf(buffer,"under-seg");
    strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%d",i);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",MSMappingMeasures[i][0]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%1.3lf",MSMappingMeasures[i][1]);
    strncat(buffer,BLANKS,FW_MEASURES-strlen(buffer));
    strcat(line,buffer);
    if (PrintFile)
      fprintf(fpt,"%s\n",line);
    (*U_Avg_m1) += MSMappingMeasures[i][0];
    (*U_Avg_m2) += MSMappingMeasures[i][1];
    (*TotalUndersegClassifications)++;
    while (j != TotalRegionsInGT+10)
      {
      LabelCount=0;
      for (k=j; k<TotalRegionsInGT+10; k++)
        {
        if(GTRegionMappings[k] == i)
	  {
	  if (LabelCount == 0)
	    sprintf(buffer,"%d",k);
	  else
            {
            sprintf(temp,",%d",k);
	    if (strlen(buffer)+strlen(temp) > FW_LABELS-1)
              break;
            strcat(buffer,temp);
            }
          LabelCount++;
	  }
        }
      if (PrintFile)
        fprintf(fpt,"%s\n",buffer);
      j=k;
      }
    }  
  }
if(*TotalUndersegClassifications > 0)
  {
  (*U_Avg_m1) /= (double)(*TotalUndersegClassifications);
  (*U_Avg_m2) /= (double)(*TotalUndersegClassifications);
  }


	/*  Print out all missed regions together  */
*TotalMissedClassifications=0;
for (i=10; (i-10)<TotalRegionsInGT; ++i)
  {
  if(GTRegionClassifications[i] == 'm')
    {
    sprintf(buffer,"%d",i);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcpy(line,buffer);
    sprintf(buffer,"missed");
    strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
    strcat(line,buffer);
    if (PrintFile)
      fprintf(fpt,"%s\n",line);
    (*TotalMissedClassifications)++;
    }
  }

	/*  Print out all noise regions together  */
*TotalNoiseClassifications=0;
for (i=10; (i-10)<TotalRegionsInMS; ++i)
  {
  if(MSRegionClassifications[i] == 'n')
    {
    strncpy(line,BLANKS,FW_LABELS);
    line[FW_LABELS]='\0';
    sprintf(buffer,"noise");
    strncat(buffer,BLANKS,FW_CLASSIFS-strlen(buffer));
    strcat(line,buffer);
    sprintf(buffer,"%d",i);
    strncat(buffer,BLANKS,FW_LABELS-strlen(buffer));
    strcat(line,buffer);
    if (PrintFile)
      fprintf(fpt,"%s\n",line);
    (*TotalNoiseClassifications)++;
    }
  }

if (!PrintFile)
  return;

	/* Print out all totals, averages and such */
fprintf(fpt,"\n\nTotal regions in Ground Truth:  %d\n",TotalRegionsInGT);
fprintf(fpt,"Total regions in Machine Segmentation:  %d\n",TotalRegionsInMS);

fprintf(fpt,"\n\nTotal CORRECT DETECTION classifications:  %d\n",
		*TotalCorrectClassifications);
if(TotalCorrectClassifications > 0)
  {
  fprintf(fpt,"\tAverage measure1:  %lf\n",*C_Avg_m1);
  fprintf(fpt,"\tAverage measure2:  %lf\n",*C_Avg_m2);
  }
else
  {
  fprintf(fpt,"\tAverage measure1:  Not applicable\n");
  fprintf(fpt,"\tAverage measure2:  Not applicable\n");
  }
fprintf(fpt,"\n");
fprintf(fpt,"\tTotal angles recorded in ground truth:  %d\n",TotalAnglesInGT);
fprintf(fpt,"\tTotal angles compared:  %d\n",*TotalAnglesCompared);
if(*TotalAnglesCompared > 0)
  {
  fprintf(fpt,"\tAverage error in compared angles (degrees) :  %lf\n",
		*AvgAngDiff);
  fprintf(fpt,"\tStandard deviation:  %lf\n\n",*stddev);
  }
else
  {
  fprintf(fpt,"\tAverage error in compared angles (degrees) :  Not applicable\n");
  fprintf(fpt,"\tStandard deviation:  Not applicable\n");
  }

fprintf(fpt,"\n\nTotal OVER-SEGMENTATION classifications:  %d\n",
		*TotalOversegClassifications);
if(*TotalOversegClassifications > 0)
  {
  fprintf(fpt,"\tAverage measure1:  %lf\n",*O_Avg_m1);
  fprintf(fpt,"\tAverage measure2:  %lf\n",*O_Avg_m2);
  }
else
  {
  fprintf(fpt,"\tAverage measure1:  Not applicable\n");
  fprintf(fpt,"\tAverage measure2:  Not applicable\n");
  }

fprintf(fpt,"\n\nTotal UNDER-SEGMENTATION classifications:  %d\n",
		*TotalUndersegClassifications);
if(*TotalUndersegClassifications > 0)
  {
  fprintf(fpt,"\tAverage measure1:  %lf\n",*U_Avg_m1);
  fprintf(fpt,"\tAverage measure2:  %lf\n",*U_Avg_m2);
  }
else
  {
  fprintf(fpt,"\tAverage measure1:  Not applicable\n");
  fprintf(fpt,"\tAverage measure2:  Not applicable\n");
  }

fprintf(fpt,"\n\nTotal MISSED classifications:  %d\n",
		*TotalMissedClassifications);
fprintf(fpt,"\n\nTotal NOISE classifications:  %d\n",
		*TotalNoiseClassifications);
fclose(fpt);

	/* DONE! */
}




int LoadFiles(FilePrefix,ROWS,COLS,GTImage,TotalRegionsInGT,
	TotalAnglesInGT,GTAngleRegions,GTAngleValues,
	MSImage,TotalRegionsInMS,MSRegionNormals, NoNormals)

char		FilePrefix[];
int		*ROWS,*COLS;
unsigned char	**GTImage,**MSImage;
int		*TotalRegionsInGT,*TotalRegionsInMS;
int		*TotalAnglesInGT;

int		GTAngleRegions[MAX_ANGLES][2];
double		GTAngleValues[MAX_ANGLES];
double		MSRegionNormals[MAX_LABELS][3];
int             *NoNormals;

{
FILE		*fpt;
char		text[180];
int		RasterHeader[8];
int		i,a;
unsigned int j;
int		GTIndex[MAX_LABELS],MSIndex[MAX_LABELS];

	/* Read in ground truth segmentation */
i=0;
while(PATHS[i])
  {
  strcpy(text, PATHS[i]);
  strcat(text, FilePrefix);
  strcat(text,".gt-seg");
  if ((fpt=fopen(text,"r")) != NULL)
    break;
  i++;
  }
if (PATHS[i] == NULL)
  {
  fprintf(stderr,"Couldn't open %s.gt-seg\n",FilePrefix);
  return(0);
  }
fread(RasterHeader,sizeof(int),8,fpt);
if (RasterHeader[0] != RAS_MAGIC)
{
	if (ntohl(RasterHeader[0]) != RAS_MAGIC){
		printf("Wrong file format for ground truth image (must be rasterfile).\n");
		exit(1);
	}
	for(j=0; j<8; ++j){
		RasterHeader[j] = ntohl(RasterHeader[j]);
	}
}
if (RasterHeader[7] != 0)
{
	fprintf(stderr,"Color format of sunraster file not supported.\n");
	exit(1);
}
*ROWS=RasterHeader[2];
*COLS=RasterHeader[1];
if ((*GTImage=(unsigned char *)calloc((*ROWS)*(*COLS), sizeof(unsigned char))) == NULL)
{
	fprintf(stderr,"Could not allocate space for array GTImage.\n");
	fprintf(stderr,"Program terminating!\n");
	exit(1);
}
fread(*GTImage,sizeof(unsigned char),(*ROWS)*(*COLS),fpt);
fclose(fpt);

if(!(*NoNormals)){
	/* Read in ground truth angles file */
i=0;
while(PATHS[i])
  {
  strcpy(text, PATHS[i]);
  strcat(text, FilePrefix);
  strcat(text,".gt-ang");
  if ((fpt=fopen(text,"r")) != NULL)
    break;
  i++;
  }
if (PATHS[i] == NULL)
  {
  fprintf(stderr,"Couldn't open %s.gt-ang\n",FilePrefix);
  return(0);
  }
fscanf(fpt,"%d",TotalAnglesInGT);
for (i=0; i<*TotalAnglesInGT; i++)
  {
  if (fscanf(fpt,"%d %d %lf",&GTAngleRegions[i][0],&GTAngleRegions[i][1],
                             &GTAngleValues[i]) != 3)
    {
    fprintf(stderr,"Wrong number of angles in %s.\n",text);
    exit(1);
    }
  }
fclose(fpt);
}
	/* Read in machine segmentation */
i=0;
while(PATHS[i])
  {
  strcpy(text, PATHS[i]);
  strcat(text, FilePrefix);
  strcat(text,".ms-seg");
  if ((fpt=fopen(text,"r")) != NULL)
    break;
  i++;
  }
if (PATHS[i] == NULL)
  {
  fprintf(stderr,"Couldn't open %s.ms-seg\n",FilePrefix);
  return(0);
  }
fread(RasterHeader,sizeof(int),8,fpt);
if (RasterHeader[0] != RAS_MAGIC)
{
	if (ntohl(RasterHeader[0]) != RAS_MAGIC){
		printf("Wrong file format for ground truth image (must be rasterfile).\n");
		exit(1);
	}
	for(j=0; j<8; ++j){
		RasterHeader[j] = ntohl(RasterHeader[j]);
	}
}
if (RasterHeader[0] == RAS_MAGIC)
  {
  if (RasterHeader[7] != 0)
    {
    fprintf(stderr,"Color format of sunraster file not supported.\n");
    exit(1);
    }
  if (RasterHeader[1] != *COLS  ||  RasterHeader[2] != *ROWS)
    {
    fprintf(stderr,"Image size does not match that of the ground truth.\n");
    exit(1);
    }
  if ((*MSImage=(unsigned char *)calloc((*ROWS)*(*COLS), sizeof(unsigned char)))
	== NULL)
    {
    fprintf(stderr,"Could not allocate space for array MSImage.\n");
    fprintf(stderr,"Program terminating!\n");
    exit(1);
    }
  fread(*MSImage,sizeof(unsigned char),(*ROWS)*(*COLS),fpt);
  fclose(fpt);
  }
else
  {
  printf("Wrong file format for machine segmentation (must be rasterfile).\n");
  exit(1);
  }

if(!(*NoNormals)){
	/* Read in normals for machine segmentation image regions */
i=0;

while(PATHS[i])
  {
  strcpy(text, PATHS[i]);
  strcat(text, FilePrefix);
  strcat(text,".ms-nor");
  if ((fpt=fopen(text,"r")) != NULL)
    break;
  i++;
  }
if (PATHS[i] == NULL)
  {
  fprintf(stderr,"Couldn't open %s.ms-nor\n",FilePrefix);
  return(0);
  }
fscanf(fpt,"%d",TotalRegionsInMS);
for (i=0; i<*TotalRegionsInMS; i++)
  {
  if (fscanf(fpt,"%d",&a) != 1)
    {
    fprintf(stderr,"Bad file format in %s.\n",text);
    exit(1);
    }
  if (fscanf(fpt,"%lf %lf %lf",&MSRegionNormals[a][0],
	&MSRegionNormals[a][1],&MSRegionNormals[a][2]) != 3)
    {
    fprintf(stderr,"Bad file format in %s.\n",text);
    exit(1);
    }
  if (MSRegionNormals[a][2] < 0.0)
    {
    MSRegionNormals[a][0]=-MSRegionNormals[a][0];
    MSRegionNormals[a][1]=-MSRegionNormals[a][1];
    MSRegionNormals[a][2]=-MSRegionNormals[a][2];
    }
  }
fclose(fpt);
}

	/* Although we already (supposedly) know TotalRegionsInMS,
	** we'll compute both it and TotalRegionsInGT explicitly here.
	** This is done by seeing how many consecutive labels there
	** are in each image from 10 upwards.  At the same time, we'll
	** do a little checking to make sure the images are proper.
	** Boy, are we nice. */
for (i=0; i<MAX_LABELS; i++)
  GTIndex[i]=MSIndex[i]=0;
for (i=0; i<(*ROWS)*(*COLS); i++)
  GTIndex[(*GTImage)[i]]=MSIndex[(*MSImage)[i]]=1;
for (i=0; i<10; i++)
  {
  if (GTIndex[i]  &&  !(NOISE_DEFINED[i]))
    {
    fprintf(stderr,"Ground truth image has badly defined pixel value %d\n",i);
    exit(1);
    }
  if (MSIndex[i]  &&  !(NOISE_DEFINED[i]))
    {
    fprintf(stderr,"Machine segmentation has badly defined pixel value %d\n",i);
    exit(1);
    }
  }
*TotalRegionsInMS=*TotalRegionsInGT=0;
for (i=10; i<MAX_LABELS; i++)
  {
  if (GTIndex[i])
    {
    (*TotalRegionsInGT)++;
    if ((i > 10  &&  !(GTIndex[i-1]))  ||  !(GTIndex[10]))
      {
      fprintf(stderr,"Ground truth region labels not consecutive\n");
      fprintf(stderr," from 10 upward.  %d is bad.\n",i);
      exit(1);
      }
    }
  if (MSIndex[i])
    {
    (*TotalRegionsInMS)++;
    if ((i > 10  &&  !(MSIndex[i-1]))  ||  !(MSIndex[10]))
      {
      fprintf(stderr,"Machine segmentation region labels not consecutive\n");
      fprintf(stderr," from 10 upward.  %d is bad.\n",i);
      exit(1);
      }
    }
  }
if (*TotalRegionsInGT == 0)
  {
   fprintf(stderr,"Nothing but noise regions in ground truth.\n");
   exit(1);
  }
if (*TotalRegionsInMS == 0)
  {
   fprintf(stderr,"Nothing but noise regions in machine segmentation.\n");
   exit(1);
  }
	/* DONE! */
return(1);
}






char digits[10][5][4]={
{
"1111",
"1  1",
"1  1",
"1  1",
"1111",
},{
" 11 ",
" 11 ",
" 11 ",
" 11 ",
" 11 ",
},{
"1111",
"   1",
"1111",
"1   ",
"1111",
},{
"1111",
"   1",
"1111",
"   1",
"1111",
},{
"1  1",
"1  1",
"1111",
"   1",
"   1",
},{
"1111",
"1   ",
"1111",
"   1",
"1111",
},{
"1   ",
"1   ",
"1111",
"1  1",
"1111",
},{
"1111",
"   1",
"   1",
"   1",
"   1",
},{
"1111",
"1  1",
"1111",
"1  1",
"1111",
},{
"1111",
"1  1",
"1111",
"   1",
"   1",
}};

unsigned char color_numbers[24]={0,0,0,103,102,135,46,169,103,60,89,234,
				215,134,29,218,2,121,86,19,123,255,255,255};

void OutputImage(Filename,InImage,TotalRegions,RegionClassifications)

char		Filename[];
unsigned char	*InImage;
int		TotalRegions;
char		RegionClassifications[MAX_LABELS];

{
unsigned char	*OutImage;
FILE		*fpt;
unsigned char	cmap[24];
int		i,j,r,c,d,x,y,width,height,real_width,raster[8];
int		CenterR[256],CenterC[256],TotalPixels[256];
int		Label[5][14],mappit[8];

if ((OutImage=(unsigned char *)calloc(ROWS*COLS,sizeof(unsigned char))) == NULL)
  {
  printf("Unable to allocate space for OutImage.\n");
  exit(1);
  }

for (i=0; i<8; i++)
  mappit[i]=0;
for (i=0; i<ROWS*COLS; i++)
  {
  if (InImage[i] < 10)
    mappit[0]=1;
  else if (RegionClassifications[InImage[i]] == 'c')
    mappit[1]=1;
  else if (RegionClassifications[InImage[i]] == 'o')
    mappit[2]=1;
  else if (RegionClassifications[InImage[i]] == 'u')
    mappit[3]=1;
  else if (RegionClassifications[InImage[i]] == 'n')
    mappit[4]=1;
  else if (RegionClassifications[InImage[i]] == 'm')
    mappit[5]=1;
  }
mappit[6]=mappit[7]=1;
for (i=7; i>=0; i--)
  if (mappit[i])
    for (j=i-1; j>=0; j--)
      mappit[i]+=mappit[j];

for (i=10; i<10+TotalRegions; i++)
  CenterR[i]=CenterC[i]=TotalPixels[i]=0;
for (r=0; r<ROWS; r++)
  for (c=0; c<COLS; c++)
    if (InImage[r*COLS+c] >= 10)
      {
      CenterR[InImage[r*COLS+c]]+=r;
      CenterC[InImage[r*COLS+c]]+=c;
      TotalPixels[InImage[r*COLS+c]]++;
      switch(RegionClassifications[InImage[r*COLS+c]])
        {
	case 'c':
	  OutImage[r*COLS+c]=mappit[1]-1;
	  break;
	case 'o':
	  OutImage[r*COLS+c]=mappit[2]-1;
	  break;
	case 'u':
	  OutImage[r*COLS+c]=mappit[3]-1;
	  break;
	case 'n':
	  OutImage[r*COLS+c]=mappit[4]-1;
	  break;
	case 'm':
	  OutImage[r*COLS+c]=mappit[5]-1;
	  break;
        default:	/* should never happen */
	  OutImage[r*COLS+c]=mappit[6]-1;
	  break;
	}
      if ((r > 0  &&  InImage[(r-1)*COLS+c] != InImage[r*COLS+c])  ||
	  (r < ROWS-1  &&  InImage[(r+1)*COLS+c] != InImage[r*COLS+c])  ||
	  (c > 0  &&  InImage[r*COLS+c-1] != InImage[r*COLS+c])  ||
	  (c < COLS-1  &&  InImage[r*COLS+c+1] != InImage[r*COLS+c]))
	OutImage[r*COLS+c]=mappit[6]-1;	/* border of region */
      }
    else
      OutImage[r*COLS+c]=mappit[0]-1;	/* unlabeled/noise pixel */

for (r=0; r<5; r++)
  for (c=0; c<14; c++)
    Label[r][c]=0;
for (i=10; i<TotalRegions+10; i++)
  {
  CenterR[i]/=TotalPixels[i];
  CenterC[i]/=TotalPixels[i];
  width=0;
  for (j=0; j<3; j++)
    {
    if (j == 0  &&  i < 100)
      continue;
    if (j == 0)
      d=i/100;
    else if (j == 1)
      d=(i-((i/100)*100))/10;
    else
      d=i-((i/10)*10);
    for (r=0; r<5; r++)
      for (c=0; c<4; c++)
        if (digits[d][r][c] != ' ')
          Label[r][c+width]=1;
        else
          Label[r][c+width]=0;
    width+=5;
    }
  width=real_width=width+4;
  height=5;
  if (!GoodSpot(InImage,ROWS,COLS,i,CenterR[i],CenterC[i],height,width))
    {
    height*=3;
    width*=3;
    while (height > 1  &&  width > 3)
      {
      for (r=0; r<ROWS; r+=height/2)
	{
	for (c=0; c<COLS; c+=width/4)
	  if (GoodSpot(InImage,ROWS,COLS,i,r,c,height,width))
	    break;
        if (c < COLS)
	  break;
	}
      if (r < ROWS)
	break;
      height/=2;
      width/=4;
      }
    if (height <= 1  ||  width <= 3)
      {
      r=CenterR[i];
      c=CenterC[i];
      }
    }
  else
    {
    r=CenterR[i];
    c=CenterC[i];
    }
  r-=2;
  c-=real_width/2;
  for (y=0; y<5; y++)
    for (x=0; x<real_width; x++)
      if (Label[y][x] == 1)
        OutImage[(r+y)*COLS+c+x]=mappit[7]-1;
  }

if ((fpt=fopen(Filename,"w")) == NULL)
  {
  printf("Unable to open %s for writing.\n",Filename);
  exit(1);
  }
i=0;
for (c=0; c<8; c++)
  if (mappit[c])
    i++;
j=0;
for (c=0; c<8; c++)
  if (mappit[c])
    {
    cmap[j]=color_numbers[c*3];
    cmap[i+j]=color_numbers[c*3+1];
    cmap[i*2+j]=color_numbers[c*3+2];
    j++;
    }
raster[0]=RAS_MAGIC;
raster[1]=COLS;
raster[2]=ROWS;
raster[3]=8;
raster[4]=ROWS*COLS;
raster[5]=1;
raster[6]=1;
raster[7]=j*3;
fwrite(raster,sizeof(int),8,fpt);
fwrite(cmap,sizeof(unsigned char),j*3,fpt);
fwrite(OutImage,sizeof(unsigned char),ROWS*COLS,fpt);
fclose(fpt);
}



int GoodSpot(InImage,ROWS,COLS,Label,r,c,height,width)

unsigned char	*InImage;
int		ROWS,COLS,Label,r,c,width,height;

{
int	x,y;

for (y=r-height/2; y<=r+height/2; y++)
  for (x=c-width/2; x<=c+width/2; x++)
    if (y < 0  ||  y >= ROWS  ||  x < 0  ||  x >= COLS  ||
	InImage[y*COLS+x] != Label)
      return(0);
return(1);
}




