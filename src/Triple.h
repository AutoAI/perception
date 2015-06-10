#pragma once 

class Triple {
	public:
		Triple();
		Triple(float x, float y, float z);
		float x;
		float y; 
		float z;
		bool operator==(const Triple &t1);
		bool operator!=(const Triple &t1);
};
