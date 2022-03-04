#pragma once
#include "Ray.h"
#include "Intersect.h"
#include "vec.h"
#include "utils.h"
#include "adept_arrays.h"
class Object {
public:
    Material* mat;
    Object() : mat(nullptr) {}
    Object(Material* m) : mat(m) {}
	virtual void intersect(Ray ray, Intersect& isec) = 0;
};

class Line : public Object {
public:
	vec2 pa;
	vec2 pb;
    Line() : pa(0.0), pb(0.0) {}
    Line(vec2 a, vec2 b, Material* m) : pa(a), pb(b), Object(m) {}
	virtual void intersect(Ray ray, Intersect& isec) override {
        vec2 sT = pb - pa;
        vec2 sN = vec2(-sT[1], sT[0]);
        double t = dot(sN, pa - ray.origin) / dot(sN, ray.dir);
        double u = dot(sT, ray.origin + ray.dir * t - pa);
        if (t < isec.tMin || t >= isec.tMax || u < 0.0 || u > dot(sT, sT))
            return;
        isec.wo = -ray.dir;
        isec.tMax = t;
        isec.n = sN.normalized();
        if (dot(ray.dir, isec.n) > 0) {
            isec.n = -isec.n;
        }
        isec.obj = this;
        isec.pos = ray.at(isec.tMax);
	}
};

class BBox : public Object {
public:
    vec2 center;
    vec2 radius;
    BBox() : center(0.0), radius(0.0) {}
    BBox(vec2 c, vec2 r, Material* m) : center(c), radius(r), Object(m) {}
    virtual void intersect(Ray ray, Intersect& isec) override {
        vec2 pos = ray.origin - center;
        double tx1 = (-radius[0] - pos[0]) / ray.dir[0];
        double tx2 = (radius[0] - pos[0]) / ray.dir[0];
        double ty1 = (-radius[1] - pos[1]) / ray.dir[1];
        double ty2 = (radius[1] - pos[1]) / ray.dir[1];

        double minX = min(tx1, tx2), maxX = max(tx1, tx2);
        double minY = min(ty1, ty2), maxY = max(ty1, ty2);

        double tmin = max(isec.tMin, max(minX, minY));
        double tmax = min(isec.tMax, min(maxX, maxY));

        if (tmax >= tmin) {
            isec.tMax = (tmin == isec.tMin) ? tmax : tmin;
            isec.n = isec.tMax == tx1 ? vec2(1.0, 0.0) : isec.tMax == tx2 ? vec2(-1.0, 0.0) :
                isec.tMax == ty1 ? vec2(0.0, 1.0) : vec2(0.0, -1.0);
            isec.obj = this;
            isec.wo = -ray.dir;
            isec.pos = ray.at(isec.tMax);
        }
    }
};

class AreaLight : public Object {
public:
    vec2 pa;
    vec2 pb;
    AreaLight() : pa(0.0), pb(0.0) {}
    AreaLight(vec2 a, vec2 b, LightMaterial* m) : pa(a), pb(b), Object(m) {}
    virtual void intersect(Ray ray, Intersect& isec) override {
        vec2 sT = pb - pa;
        vec2 sN = vec2(-sT[1], sT[0]);
        double t = dot(sN, pa - ray.origin) / dot(sN, ray.dir);
        double u = dot(sT, ray.origin + ray.dir * t - pa);
        if (t < isec.tMin || t >= isec.tMax || u < 0.0 || u > dot(sT, sT))
            return;

        isec.tMax = t;
        isec.n = sN.normalized();
        if (dot(ray.dir, isec.n) > 0) {
            isec.n = -isec.n;
        }
        isec.obj = this;
        isec.wo = -ray.dir;
        isec.pos = ray.at(isec.tMax);
    }

    Ray sample_Le(double& pdfPos, double& pdfDir, vec2& n, vec3& color) {
        vec2 sT = pb - pa;
        vec2 sN = vec2(-sT[1], sT[0]);
        double k = uniform();
        vec2 origin = pa * k + pb * (1 - k);
        double x = uniform() * M_PI;
        vec2 dirLocal = vec2(cos(x), sin(x));
        pdfPos = 1.0 / sT.length();
        pdfDir = 1.0 / M_PI;
        //if (dirLocal[1] < 1e-5) {
        //    dirLocal = vec2(0.0, 1.0);
        //    pdfDir = 0.5;
        //}
        n = sN.normalized();
        color = ((LightMaterial*)mat)->Le(dirLocal);
        vec2 dir = dirLocal[0] * sT.normalized() + dirLocal[1] * sN.normalized();
        return Ray(origin, dir);
    }

    vec3 sample_Li(vec2 pos, vec2& u, double& pdf, vec2& wi) {
        vec2 sT = pb - pa;
        vec2 sN = vec2(-sT[1], sT[0]).normalized();
        double k = uniform();
        u = pa * k + pb * (1 - k);
        vec2 dir = (pos - u).normalized();
        vec2 dirLocal = vec2(dot(dir, sT.normalized()), dot(dir, sN));
        pdf = (u - pos).length() / (abs(dirLocal[1]) * sT.length());
        wi = dir;
        vec3 color = ((LightMaterial*)mat)->Le(dirLocal);
        // dir = dir[0] * sT.normalized() + dir[1] * sN.normalized();
        return color;
    }
};

