#pragma once 

class Triple {
	public:
		float x;
		float y; 
		float z;
		friend bool operator==(const Triple &t1, const Triple &t2);
};
