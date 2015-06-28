#pragma once

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef enum {red,blue,green,yellow} colour_t;
typedef enum {square_sh,circle_sh,triangle_sh} shape_t;

class Shape
{
public:
	Shape(void);
	Shape(int xPos, int yPos, colour_t c, shape_t s);
	~Shape(void);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	colour_t getColour();
	void setColour(colour_t c);

	shape_t getShape();
	void setShape(shape_t s);

	int getRadius();
	void setRadius(int r);


private:
	int xPos, yPos;
	colour_t colour;
	shape_t shape;
	int radius;
};

