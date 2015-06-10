#pragma once 

#include "Triple.h"

bool Triple::operator==(const Triple& t) {
	return (t.x == x && t.y == y && t.z == z);
}
