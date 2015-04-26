#pragma once

#include <vector> 
#include "CoordinateList.h"

using namespace std;

void CoordinateList::addCoordinate(Coordinate* coordiante){
    coordinates.push_back(coordinate);
}

void CoordinateList::toType(char newType) {
    this.toType(newType, {0,0,0}); 
}

void CoordinateList::toCartesian(){
    if(type==Cartesian){
        return;
    }
    for(std::vector<T>::iterator it = v.begin(); it != v.end(); ++it) {
     int x,y,z;  
        if(type==Spherical){
            r = it[0];
            theta =  it[1];
            roe = it[2];
            
            x = r*sin(roe)*cos(theta);
            y = r*sin(roe)*cos(theta);
            z = r*cos(roe);
        }
        if(type==Perspective){
            u = it[0];
            v = it[1];
            w = it[2];

            z = K/w;
            y = v*z/F;
            x = u*z/F;
        }
    it[0] = x;
    it[1] = y;
    it[2] = z;
    }
    type = Cartesian;
}

void CoordinateList::toType(char newType, const Coordinate* offset) {
    if(newType==type){
        return;
    }
    this.toCartesian();
    for(std::vector<T>::iterator it = v.begin(); it != v.end(); ++it) {
       
        int x = it[0] + offset[0];
        int y = it[1] + offset[1];
        int z = it[2] + offset[2];
        
        if(newType==Spherical){
            r = sqrt(x^2+y^2+z^2);
            theta = atan(y/z);
            roe = atan(z/r);

            it[0] = r;
            it[1] = theta;
            it[2] = roe;
        }

        if(newType==Perspective){
            u = x*F/z;
            v = y*F/z;
            w = K/z;
           
            it[0] = u;
            it[1] = v;
            it[2] = w;
        }
    }
    type = newType;
}

CopordinateList clone(){
    CoordinateList ret;
    for(std::vector<T>::iterator it = v.begin(); it != v.end(); ++it) {
        ret.addCoordinate(it);
    }
    return ret;
}

