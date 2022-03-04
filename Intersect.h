#pragma once
#include "vec.h"
#include "Material.h"
class Object;

class Intersect
{
public:
	double tMin = 1e-5;
	double tMax = 1e30;
	Object* obj = nullptr;
	vec2 n = vec2(0.0, 0.0);
	vec2 wo = vec2(1.0, 0.0);
	vec2 pos;
};

