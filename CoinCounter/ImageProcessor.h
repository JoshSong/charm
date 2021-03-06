#ifndef _IMAGEPROCESSOR_H_
#define _IMAGEPROCESSOR_H_

#include <vector>

#include <windows.h>

LONGLONG millisecondsNow();

enum COIN_TYPES { _5_CENT, _10_CENT, _20_CENT, _50_CENT, _1_DOLLAR, _2_DOLLAR, NUM_COIN_TYPES };

struct Object
{
	float x, y, diameter;
	int type;
};

class ImageProcessor
{
public:

	enum MODE { MODE_NORMAL, MODE_CALIBRATE, MODE_TRAINCLASSIFY };

	void calibrate();
	LONGLONG run(int mode, int numFrames, std::vector<Object> &coins, float &tableSpeed);

private:
	double findMean(std::vector<double> list);
	double findStdDev(std::vector<double> list);
	double findMeanNoOutliers(std::vector<double> list);

};


#endif //_IMAGEPROCESSOR_H_