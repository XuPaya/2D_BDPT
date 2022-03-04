#pragma once
#include <iostream>
#include "Ray.h"
#include "Intersect.h"
#include <vector>
#include "Object.h"
using namespace std;

class Scene
{
public:
    vec2 camera_pos;
    vector<Object*> objs;// this is the boundary of our 2D scene. the name is misleading...
    AreaLight* arealight;
    Scene() {
        LambertianMaterial* lambertian = new LambertianMaterial();
        Line* line0 = new Line(vec2(-0.3, 0.8), vec2(0.3, 0.4), lambertian);
        Line* line1 = new Line(vec2(0.16, -0.9), vec2(1.0, -0.2), lambertian);
        BBox* bbox = new BBox(vec2(0.0, 0.0), vec2(1.78, 1.0), lambertian);
        LightMaterial* light_mat = new LightMaterial();
        AreaLight* light = new AreaLight(vec2(-1.78 + 1e-10, 0.2), vec2(-1.78 + 1e-10, -0.2), light_mat);
        objs.push_back(bbox);
        objs.push_back(line0);
        objs.push_back(line1);
        arealight = light;
    }

    void intersect(Ray& ray, Intersect& isec) const {
        for (int idx = 0; idx < objs.size(); idx++)
            objs[idx]->intersect(ray, isec);
        arealight->intersect(ray, isec);
	}
};

