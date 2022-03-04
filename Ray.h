#pragma once
#include "vec.h"
class Ray
{
public:
	Ray(vec2 ori, vec2 d): origin(ori), dir(d){}
	vec2 origin;
	vec2 dir;
	vec2 at(double t) {
		return origin + dir * t;
	}
};

