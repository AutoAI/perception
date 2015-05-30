#pragma once

#include <vector> 
#include "CoordinateList.h"

using namespace std;

CoordinateList::CoordinateList(char type) {
    this->type = type;
}

void CoordinateList::addCoordinate(Coordinate* coordinate){
    coordinates.push_back(coordinate);
}

void CoordinateList::toType(char newType) {
	Coordinate d[] = {0, 0, 0};
	
    this->toType(newType, d); 
}

void CoordinateList::toCartesian(){
    if(type==Cartesian){
        return;
    }
	Coordinate x,y,z;  
	Coordinate r, theta, roe;
	Coordinate u, v, w;

    for(int i = 0; i < coordinates.size(); i++) {
        if(type==Spherical){
            r = coordinates[i][0];
            theta =  coordinates[i][1];
            roe = coordinates[i][2];
            
            x = r*sin(roe)*cos(theta);
            y = r*sin(roe)*cos(theta);
            z = r*cos(roe);
        }
        if(type==Perspective){
            u = coordinates[i][0];
            v = coordinates[i][1];
            w = coordinates[i][2];

            z = K/w;
            y = v*z/F;
            x = u*z/F;
        }
        coordinates[i][0] = x;
        coordinates[i][1] = y;
        coordinates[i][2] = z;
    }
    type = Cartesian;
}

void CoordinateList::toType(char newType, Coordinate* offset) {
	Coordinate r, theta, roe;
	Coordinate u, v, w;
	Coordinate x, y, z;
    if(newType==type){
        return;
    }
    this->toCartesian();
    for(int i = 0; i < coordinates.size(); i++){
       
        x = coordinates[i][0] + offset[0];
        y = coordinates[i][1] + offset[1];
        z = coordinates[i][2] + offset[2];
        
        if(newType==Spherical){
            r = sqrt(x*x + y*y + z*z);
            theta = atan(y/z);
            roe = atan(z/r);

            coordinates[i][0] = r;
            coordinates[i][1] = theta;
            coordinates[i][2] = roe;
        }

        if(newType==Perspective){
            u = x*F/z;
            v = y*F/z;
            w = K/z;
           
            coordinates[i][0] = u;
            coordinates[i][1] = v;
            coordinates[i][2] = w;
        }
    }
    type = newType;
}

Coordinate* CoordinateList::getCoordinate(int index) {
    return coordinates[index];
}

CoordinateList CoordinateList::clone(){
    CoordinateList ret(this->type);
    for(int i = 0; i < coordinates.size(); i++) {
        ret.addCoordinate(coordinates[i]);
    }
    return ret;
}

