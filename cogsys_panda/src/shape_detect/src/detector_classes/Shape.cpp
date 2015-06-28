#include "detector_classes/Shape.h"


Shape::Shape(void)
{
}

Shape::Shape(int xPos, int yPos, colour_t c, shape_t s) {
	this->xPos = xPos;
	this->yPos = yPos;
	this->colour = c;
	this->shape = s;
}


Shape::~Shape(void)
{
}

int Shape::getXPos() {
	return this->xPos;
}

void Shape::setXPos(int x) {
	this->xPos = x;
}

int Shape::getYPos() {
	return this->yPos;
}

void Shape::setYPos(int y) {
	this->yPos = y;
}

colour_t Shape::getColour() {
	return this->colour;
}

void Shape::setColour(colour_t c) {
	this->colour = c;
}

shape_t Shape::getShape() {
	return this->shape;
}

void Shape::setShape(shape_t s) {
	this->shape = s;
}
