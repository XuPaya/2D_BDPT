#pragma once
#include "vec.h"
#include "utils.h"

enum class matType {Light, Object};

class Material
{
public:
	matType type;
	virtual vec2 sample(vec2 wi) = 0;
	virtual vec3 bsdf(vec2 wi, vec2 wo) = 0;
	virtual double pdf(vec2 wi, vec2 wo) = 0;
};

class LambertianMaterial: public Material {
public:
	vec3 color;
	LambertianMaterial() :color(0.6) { type = matType::Object; }
	LambertianMaterial(vec3 c) :color(c) { type = matType::Object; }
	LambertianMaterial(double r, double g, double b) :color(r, g, b) { type = matType::Object; }
	virtual vec2 sample(vec2 wi) {
		double theta = uniform() * M_PI;
		double x = cos(theta);
		double y = sin(theta);
		return vec2(x, -y * sign(wi[1]));
	}
	virtual vec3 bsdf(vec2 wi, vec2 wo) {
		return vec3(0.3);
	}
	virtual double pdf(vec2 wi, vec2 wo) {
		return 1.0 / M_PI;
	}


};

class LightMaterial: public Material {
public:
	vec3 color;
	LightMaterial():color(1.0) { type = matType::Light; }
	LightMaterial(vec3 c) :color(c) { type = matType::Light; }
	LightMaterial(double r, double g, double b) :color(r, g, b) { type = matType::Light; }
	virtual vec2 sample(vec2 wi) {
		double theta = uniform() * M_PI;
		double x = cos(theta);
		double y = sin(theta);
		return vec2(x, -y * sign(wi[1]));
	}
	virtual vec3 bsdf(vec2 wi, vec2 wo) {
		return vec3(0.3);
	}
	virtual double pdf(vec2 wi, vec2 wo) {
		return 1.0 / M_PI;
	}
	vec3 Le(vec2 wi) {
		// if (abs(wi[1]) == 0) return 0.0;
		return vec3(2.2);
	}
	double Pdf_Le(vec2 wi) const {
		return 1.0 / M_PI;
	}
};


