#pragma once
#include "vec.h"
#include "avec.h"
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

class aRay
{
public:
	aRay(aVec2 ori, aVec2 d): origin(ori), dir(d){}
	aRay(Ray r) : origin(r.origin), dir(r.dir) {}
	aVec2 origin;
	aVec2 dir;
};

