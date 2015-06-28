#include "detector_classes/HSV_Param.h"
#include "main.h"

HSV_Param::HSV_Param(void)
{
}

HSV_Param::HSV_Param(colour_t c) {

	setColour(c);
	switch(c) {
	case red:
		setHSVmin(Scalar(RED_H_MIN,RED_S_MIN,RED_V_MIN));
		setHSVmax(Scalar(RED_H_MAX,RED_S_MAX,RED_V_MAX));
		break;

	case green:
		setHSVmin(Scalar(GREEN_H_MIN,GREEN_S_MIN,GREEN_V_MIN));
		setHSVmax(Scalar(GREEN_H_MAX,GREEN_S_MAX,GREEN_V_MAX));
		break;

	case blue:
		setHSVmin(Scalar(BLUE_H_MIN,BLUE_S_MIN,BLUE_V_MIN));
		setHSVmax(Scalar(BLUE_H_MAX,BLUE_S_MAX,BLUE_V_MAX));
		break;

	case yellow:
		setHSVmin(Scalar(YELLOW_H_MIN,YELLOW_S_MIN,YELLOW_V_MIN));
		setHSVmax(Scalar(YELLOW_H_MAX,YELLOW_S_MAX,YELLOW_V_MAX));
		break;

	default:
		break;
	}	
}

HSV_Param::~HSV_Param(void)
{
}



colour_t HSV_Param::getColour() {
	return this->colour;
}

void HSV_Param::setColour(colour_t c) {
	this->colour = c;
}

Scalar HSV_Param::getHSVmin() {
	return this->HSVmin;
}
Scalar HSV_Param::getHSVmax() {
	return this->HSVmax;
}

void HSV_Param::setHSVmin(Scalar min) {
	this->HSVmin = min;
}

void HSV_Param::setHSVmax(Scalar max) {
	this->HSVmax = max;
}
