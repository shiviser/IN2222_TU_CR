#pragma once

#include <opencv2/opencv.hpp>
#include "detector_classes/Shape.h"

using namespace std;
using namespace cv;

class HSV_Param
{
public:
	HSV_Param(void);
	HSV_Param(colour_t c);
	~HSV_Param(void);

	colour_t getColour();
	void setColour(colour_t c);

	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);

private:
	Scalar HSVmin, HSVmax;
	colour_t colour;

};

