

#ifndef AABB_H
#define AABB_H

#include "Vector3.h"
#include "Point.h"

#include <math.h>
#include <algorithm>

using std::min;
using std::max;
using std::endl;

class AABB{

public:
	Vector3 min;
	Vector3 max;
	Vector3 size;

	AABB(){
		min = Vector3(std::numeric_limits<double>::max());
		max = Vector3(-std::numeric_limits<double>::max());
		size = Vector3(std::numeric_limits<double>::max());
	}

	AABB(Vector3 min, Vector3 max){
		this->min = min;
		this->max = max;
		size = max-min;
	}

	bool isInside(const Point &p){
		if(min.x <= p.x && p.x <= max.x){
			if(min.y <= p.y && p.y <= max.y){
				if(min.z <= p.z && p.z <= max.z){
					return true;
				}
			}
		}

		return false;
	}

	void update(Vector3 &point){
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);

		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);
	}

	friend ostream &operator<<( ostream &output,  const AABB &value ){ 
		output << "min: " << value.min << endl;
		output << "max: " << value.max << endl;
		output << "size: " << value.size << endl;
		return output;            
	}

};



#endif