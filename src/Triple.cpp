#pragma once 

#include "Triple.h"

Triple::Triple() {}
Triple::Triple(float x, float y, float z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

bool Triple::operator==(const Triple &t1) {
	return (t1.x == x && t1.y == y && t1.z == z);
}

bool Triple::operator!=(const Triple &t1) {
	return !(t1.x == x && t1.y == y && t1.z == z);
}
