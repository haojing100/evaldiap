// evaldisp.cpp 

// evaluate disparity map
// simple version for SDK
// supports upsampling of disp map if GT has higher resolution

// DS 7/2/2014
// 10/14/2014 changed computation of average error
// 1/27/2015 added clipping of valid (non-INF) disparities to [0 .. maxdisp]
//    in fairness to those methods that do not utilize the given disparity range
//    (maxdisp is specified at disp resolution, NOT GT resolution)

static const char *usage = "\n  usage: %s disp.pfm gtdisp.pfm badthresh maxdisp rounddisp [mask.png]\n\
    (or call with single argument badthresh to print column headers)\n";

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "imageLib/imageLib.h"
#include "imageLib/Convert.h"
using namespace std;

int verbose = 0;

void evaldisp_all(CFloatImage disp, CFloatImage gtdisp,  float badthresh, int maxdisp, int rounddisp)
{
    CShape sh = gtdisp.Shape();
    CShape sh2 = disp.Shape();
    int width = sh.width, height = sh.height;
    int width2 = sh2.width, height2 = sh2.height;
    int scale = width / width2;

    if ((!(scale == 1 || scale == 2 || scale == 4))
	|| (scale * width2 != width)
	|| (scale * height2 != height)) {
	printf("   disp size = %4d x %4d\n", width2, height2);
	printf("GT disp size = %4d x %4d\n", width,  height);
	throw CError("GT disp size must be exactly 1, 2, or 4 * disp size");
    }

    int n = 0;
    int bad = 0;
    int invalid = 0;
    float serr = 0;
    for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	    float gt = gtdisp.Pixel(x, y, 0); //得到PixelAddress的内容，PixelAddress以byte为单位
	    if (gt == INFINITY) // unknown IEEE正无穷大
		continue;
	    float d = scale * disp.Pixel(x / scale, y / scale, 0);
	    int valid = (d != INFINITY);
	    if (valid) {
		float maxd = scale * maxdisp; // max disp range
		d = __max(0, __min(maxd, d)); // clip disps to max disp range 将disps剪辑到最大disp范围
	    }
	    if (valid && rounddisp)
		d = round(d);
	    float err = fabs(d - gt);
	    
		n++;
		if (valid) {
		    serr += err;
		    if (err > badthresh) { //bad threshold
			bad++;
		    }
		} else {// invalid (i.e. hole in sparse disp map)
		    invalid++;
		}
	}
    }
    float badpercent =  100.0*bad/n;
    float invalidpercent =  100.0*invalid/n;
    float totalbadpercent =  100.0*(bad+invalid)/n;
    float avgErr = serr / (n - invalid); // CHANGED 10/14/2014 -- was: serr / n
    //printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
    printf("%4.1f  %6.2f  %6.2f   %6.2f  %6.2f\n",   100.0*n/(width * height), 
	   badpercent, invalidpercent, totalbadpercent, avgErr);
}

void evaldisp_nonocc(CFloatImage disp, CFloatImage gtdisp, CByteImage mask, float badthresh, int maxdisp, int rounddisp)
{
    CShape sh = gtdisp.Shape();
    CShape sh2 = disp.Shape();
    CShape msh = mask.Shape();
    int width = sh.width, height = sh.height;
    int width2 = sh2.width, height2 = sh2.height;
    int scale = width / width2;

    if ((!(scale == 1 || scale == 2 || scale == 4))
	|| (scale * width2 != width)
	|| (scale * height2 != height)) {
	printf("   disp size = %4d x %4d\n", width2, height2);
	printf("GT disp size = %4d x %4d\n", width,  height);
	throw CError("GT disp size must be exactly 1, 2, or 4 * disp size");
    }

    int usemask = (msh.width > 0 && msh.height > 0);
    if (usemask && (msh != sh))
	throw CError("mask image must have same size as GT\n");

    int n = 0;
    int bad = 0;
    int invalid = 0;
    float serr = 0;
    for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	    float gt = gtdisp.Pixel(x, y, 0); //得到PixelAddress的内容，PixelAddress以byte为单位
	    if (gt == INFINITY) // unknown IEEE正无穷大
		continue;
	    float d = scale * disp.Pixel(x / scale, y / scale, 0);
	    int valid = (d != INFINITY);
	    if (valid) {
		float maxd = scale * maxdisp; // max disp range
		d = __max(0, __min(maxd, d)); // clip disps to max disp range 将disps剪辑到最大disp范围
	    }
	    if (valid && rounddisp)
		d = round(d);
	    float err = fabs(d - gt);
	    if (usemask && mask.Pixel(x, y, 0) != 255) { // don't evaluate pixel  mask.Pixel(x, y, 0) != 255???
	    } else {
		n++;
		if (valid) {
		    serr += err;
		    if (err > badthresh) { //bad threshold
			bad++;
		    }
		} else {// invalid (i.e. hole in sparse disp map)
		    invalid++;
		}
	    }
	}
    }
    float badpercent =  100.0*bad/n;
    float invalidpercent =  100.0*invalid/n;
    float totalbadpercent =  100.0*(bad+invalid)/n;
    float avgErr = serr / (n - invalid); // CHANGED 10/14/2014 -- was: serr / n
    //printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
    printf("%4.1f  %6.2f  %6.2f   %6.2f  %6.2f\n",   100.0*n/(width * height), 
	   badpercent, invalidpercent, totalbadpercent, avgErr);
}

vector<vector<int> > discoun_domain(CFloatImage gtdisp,int minDiscoun)
{
	CShape sh = gtdisp.Shape();
	int width = sh.width, height = sh.height;
    vector< vector<int> > disc_domain;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
	    	float gt = gtdisp.Pixel(x, y, 0); //得到PixelAddress的内容，PixelAddress以byte为单位
			float gt_xadd1 = gtdisp.Pixel(x+1, y, 0); 
			float gt_yadd1 = gtdisp.Pixel(x, y+1, 0); 
	    	if (gt == INFINITY) // unknown IEEE正无穷大
			continue;
            if(x+1<width && fabs(gt_xadd1-gt)>minDiscoun)
            {
                vector <int> t1,t2;
                t1.push_back(x);
                t1.push_back(y);
                disc_domain.push_back(t1);
                t2.push_back(x+1);
                t2.push_back(y);
                disc_domain.push_back(t2);
            }
            if(y+1<height && fabs(gt_yadd1-gt)>minDiscoun)
            {
                vector <int> t1,t2;
                t1.push_back(x);
                t1.push_back(y);
                disc_domain.push_back(t1);
                t2.push_back(x);
                t2.push_back(y+1);
                disc_domain.push_back(t2);
            }
        }
    }
	sort(disc_domain.begin(),disc_domain.end());
    disc_domain.erase(unique(disc_domain.begin(), disc_domain.end()), disc_domain.end());
	return disc_domain;
}
void evaldisp_discoun(CFloatImage disp, CFloatImage gtdisp, float badthresh,int minDiscoun,int maxdisp, int rounddisp)
{
	vector< vector<int> > disc_domain=discoun_domain(gtdisp,minDiscoun);
	CShape sh = gtdisp.Shape();
    CShape sh2 = disp.Shape();
    int width = sh.width, height = sh.height;
    int width2 = sh2.width, height2 = sh2.height;
    int scale = width / width2;

    if ((!(scale == 1 || scale == 2 || scale == 4))
	|| (scale * width2 != width)
	|| (scale * height2 != height)) {
	printf("   disp size = %4d x %4d\n", width2, height2);
	printf("GT disp size = %4d x %4d\n", width,  height);
	throw CError("GT disp size must be exactly 1, 2, or 4 * disp size");
    }

    int n = 0;
    int bad = 0;
    int invalid = 0;
    float serr = 0;
	int dd_len=disc_domain.size();
	int i=0;
	int x,y;
	for(;i<dd_len;i++)
	{
		x=disc_domain[i][0];
		y=disc_domain[i][1];
		float gt = gtdisp.Pixel(x, y, 0); //得到PixelAddress的内容，PixelAddress以byte为单位
	    if (gt == INFINITY) // unknown IEEE正无穷大
		continue;
	    float d = scale * disp.Pixel(x / scale, y / scale, 0);
	    int valid = (d != INFINITY);
	    if (valid) {
		float maxd = scale * maxdisp; // max disp range
		d = __max(0, __min(maxd, d)); // clip disps to max disp range 将disps剪辑到最大disp范围
	    }
	    if (valid && rounddisp)
		d = round(d);
	    float err = fabs(d - gt);
	    
		n++;
		if (valid) {
		    serr += err;
		    if (err > badthresh) { //bad threshold
			bad++;
		    }
		} else {// invalid (i.e. hole in sparse disp map)
		    invalid++;
		}
	}
    
    float badpercent =  100.0*bad/n;
    float invalidpercent =  100.0*invalid/n;
    float totalbadpercent =  100.0*(bad+invalid)/n;
    float avgErr = serr / (n - invalid); // CHANGED 10/14/2014 -- was: serr / n
    //printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
    printf("%4.1f  %6.2f  %6.2f   %6.2f  %6.2f\n",   100.0*n/(width * height), 
	   badpercent, invalidpercent, totalbadpercent, avgErr);
}

int main(int argc, char *argv[])
{
    try {
	int requiredargs = 4;
	int optionalargs = 3;
	if (argc >= requiredargs + 1 && argc <= requiredargs + optionalargs + 1) {
	    int argn = 1;
	    char *dispname = argv[argn++]; 
	    char *gtdispname = argv[argn++];
	    float badthresh = atof(argv[argn++]); // atof把字符串转换成浮点数
		int minDiscoun=atof(argv[argn++]);//evaldisp_discoun()中的不连续阈值
	    int maxdisp = 99999;  //maxdisp以disp分辨率指定，而不是GT分辨率 
		//此处argn=4
	    if (argc > argn)
		maxdisp = atoi(argv[argn++]); //函数说明编辑参数nptr字符串，如果第一个非空格字符存在，是数字或者正负号则开始做类型转换，之后检测到非数字(包括结束符 \0) 字符时停止转换，返回整型数。否则，返回零，
	    int rounddisp = 0;
	    if (argc > argn)
		rounddisp = atoi(argv[argn++]);
	    char *maskname = NULL;
	    if (argc > argn)
		maskname = argv[argn++];
      
	    CFloatImage disp, gtdisp, gtdisp1;
	    ReadImageVerb(disp, dispname, verbose);
	    ReadImageVerb(gtdisp, gtdispname, verbose);
	    CByteImage mask;
	    if (maskname)
		ReadImageVerb(mask, maskname, verbose);

		evaldisp_all(disp, gtdisp,badthresh,maxdisp,rounddisp);
	    evaldisp_nonocc(disp, gtdisp, mask, badthresh, maxdisp, rounddisp);
		evaldisp_discoun(disp,gtdisp, badthresh,minDiscoun,maxdisp, rounddisp);

	} else if (argc == 2) {
	    float badthresh = atof(argv[1]);
	    // show what the header looks like (can grep in scripts)
	    printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
	} else
	    throw CError(usage, argv[0]);
    }
    catch (CError &err) {
	fprintf(stderr, err.message);
	fprintf(stderr, "\n");
	return -1;
    }
  
    return 0;
}
