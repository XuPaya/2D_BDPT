#pragma once
#include "Scene.h"
#include "Vertex.h"
#include <vector>
#include "adept.h"

#define MAX_CAMERA_DEPTH 5
#define MAX_LIGHT_DEPTH 5
#define TTL_MAX_DEPTH 12


enum class TransportMode { Importance, Radiance };

template <typename Type>
class ScopedAssignment {
public:
	// ScopedAssignment Public Methods
	ScopedAssignment(Type* target = nullptr, Type value = Type())
		: target(target) {
		if (target) {
			backup = *target;
			*target = value;
		}
	}
	~ScopedAssignment() {
		if (target) *target = backup;
	}
	ScopedAssignment(const ScopedAssignment&) = delete;
	ScopedAssignment& operator=(const ScopedAssignment&) = delete;
	ScopedAssignment& operator=(ScopedAssignment&& other) {
		if (target) *target = backup;
		target = other.target;
		backup = other.backup;
		other.target = nullptr;
		return *this;
	}

private:
	Type* target, backup;
};

double visibility_test(const Scene& scene, const Vertex& v0, const Vertex& v1) {
	if (dot(v0.n(), v1.n()) > 0.0)
		return 0.0;
	if (abs(dot(v0.p() - v1.p(), v0.n())) < 1e-5) {
		return 0.0;
	}
	if (abs(dot(v0.p() - v1.p(), v1.n())) < 1e-5) {
		return 0.0;
	}
	Ray ray = Ray(v0.p(), (v1.p() - v0.p()).normalized());
	Intersect isec;
	scene.intersect(ray, isec);
	if ((isec.pos - v1.p()).length() > 1e-10)
	{
		return 0.0;
	}
	return 1.0;
}

vec3 G(const Scene& scene, const Vertex& v0, const Vertex& v1) {
	vec2 d = v0.p() - v1.p();
	double g = 1.0 / d.length();
	d = d.normalized();
	if (v0.type != VertexType::Camera)
		g *= abs(dot(v0.n(), d));
	if (v1.type != VertexType::Camera)
		g *= abs(dot(v1.n(), d));
	return g;
}

double MISWeight(const Scene& scene, Vertex* lightVertices,
	Vertex* cameraVertices, Vertex& sampled, int s, int t) {
	if (s + t == 2)
		return 1;
	double sumRi = 0.0;
	// << Define helper function remap0 that deals with Dirac delta functions >>
	auto remap0 = [](double f) -> double { return f != 0 ? f : 1; };

	// << Temporarily update vertex properties for current strategy>>
	//	<<Look up connection vertices and their predecessors >>
	Vertex* qs = s > 0 ? &lightVertices[s - 1] : nullptr,
		* pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
		* qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
		* ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

	// << Update sampled vertex for or strategy >>
	ScopedAssignment<Vertex> a1;
	if (s == 1)      a1 = { qs, sampled };
	else if (t == 1) a1 = { pt, sampled };

	// << Mark connection vertices as non - degenerate >>
	ScopedAssignment<bool> a2, a3;
	bool doll;
	if (pt) a2 = { &doll, false };
	if (qs) a3 = { &doll, false };

	//<< Update reverse density of vertex >>
	ScopedAssignment<double> a4;
	if (pt)
		a4 = { &pt->pdfRev,
			   s > 0 ? qs->Pdf(scene, qsMinus, *pt) :
					   pt->PdfLightOrigin(scene, *ptMinus) };

	// << Update reverse density of vertex >>
	ScopedAssignment<double> a5;
	if (ptMinus)
		a5 = { &ptMinus->pdfRev,
			   s > 0 ? pt->Pdf(scene, qs, *ptMinus) :
					   pt->PdfLight(scene, *ptMinus) };

	// << Update reverse density of verticesand >>

	ScopedAssignment<double> a6;
	if (qs) a6 = { &qs->pdfRev, 
		pt->Pdf(scene, ptMinus, *qs) };
	ScopedAssignment<double> a7;
	if (qsMinus) a7 = { &qsMinus->pdfRev, 
		qs->Pdf(scene, pt, *qsMinus) };

		// <<Consider hypothetical connection strategies along the camera subpath >>
	double ri = 1.0;
	for (int i = t - 1; i > 1; --i) {
		ri *= remap0(cameraVertices[i].pdfRev) /
			remap0(cameraVertices[i].pdfFwd);
		if (ri != ri) {
			return 1.0;
		}
		sumRi += ri;
	}

	// << Consider hypothetical connection strategies along the light subpath >>
	ri = 1;
	for (int i = s - 1; i >= 0; --i) {
		ri *= remap0(lightVertices[i].pdfRev) /
			remap0(lightVertices[i].pdfFwd);
		if (ri != ri)
			return 1.0;
		sumRi += ri;
	}
	return 1.0 / (1.0 + sumRi);
}

vec3 evalPath(const Scene& scene, vector<Vertex*> path) { // Path Integral, all gradient are calculated in this function
	vec3 L = path[0]->Le(*path[1]) * G(scene, *path[0], *path[1]) / path[0]->pdfFwd;

	for (int i = 1; i < path.size() - 1; i++)
		L *= path[i]->f(*path[i - 1], *path[i + 1]) * G(scene, *path[i], *path[i + 1]) / path[i]->pdfFwd;

	L *= path[path.size() - 1]->We(*path[path.size() - 2]);
	return L;
}

vec3 ConnectBDPT(const Scene& scene, Vertex* lightVertices,
	Vertex* cameraVertices, int s, int t) {
	vec3 L(0.f);
	//adept::Stack stack;
	//stack.new_recording();
	// << Perform connection and write contribution to L >>
	Vertex sampled;
	vector<Vertex*> path;
	if (s == 0) {
		// << Interpret the camera subpath as a complete path >>
		const Vertex & pt = cameraVertices[t - 1];
		if (pt.obj == scene.arealight) {
			for (int i = t - 1; i >= 0; i--) {
				path.push_back(&cameraVertices[i]);
			}
		}

	} // camera is not connectible, so ignore t == 1
	else if (s == 1) {
		// << Sample a point on a light and connect it to the camera subpath >>
		const Vertex & pt = cameraVertices[t - 1], & ptMinus = cameraVertices[t - 2];
		double vis = 1.0;
		vec2 wi; 
		double pdf;
		AreaLight* light = scene.arealight;
		vec2 u;
		vec3 lightWeight = light->sample_Li(pt.p(), u, pdf, wi);
		if (pdf > 0) {
			sampled.normal = vec2(1.0, 0.0);
			sampled.pos = u;
			sampled.type = VertexType::Light;
			sampled.pdfFwd = sampled.PdfLightOrigin(scene, pt);
			vis = visibility_test(scene, sampled, pt);
			if (vis == 0.0) {
				return 0.0;
			}
			path.push_back(&sampled);
			for (int i = t - 1; i >= 0; i--) {
				path.push_back(&cameraVertices[i]);
			}
		}
	}
	else {
		// << Handle all other bidirectional connection cases >>
		const Vertex & qs = lightVertices[s - 1], &pt = cameraVertices[t - 1], &qsMinus = lightVertices[s - 2], &ptMinus = cameraVertices[t - 2];
		double vis = visibility_test(scene, qs, pt);
		if (vis == 0.0) {
			return 0.0;
		}
		for (int i = 0; i < s; i++) {
			path.push_back(&lightVertices[i]);
		}
		for (int i = t - 1; i >= 0; i--) {
			path.push_back(&cameraVertices[i]);
		}
	}
	if (path.size() != 0) {
		L = evalPath(scene, path);
	}
	double misWeight = MISWeight(scene, lightVertices, cameraVertices, sampled, s, t);
	L *= misWeight;
	return L;
}

int RandomWalk(const Scene& scene, Ray ray, vec3 beta, double pdf, int maxDepth,
	TransportMode mode, Vertex* path) {
	if (maxDepth == 0)
		return 0;
	int bounces = 0;
	double pdfFwd = pdf, pdfRev = 0;

	while (true) {
		Intersect isect;
		scene.intersect(ray, isect);
		if (beta.length() < 1e-10)
			break;
		
		Vertex& vertex = path[bounces], & prev = path[bounces - 1];

		if (isect.tMax == 1e30) {
			break;
		}

		vertex.pos = ray.at(isect.tMax);
		vertex.normal = isect.n;
		vertex.obj = isect.obj;
		vertex.type = VertexType::Surface;
		vertex.pdfFwd = prev.ConvertDensity(pdfFwd, vertex);

		if (++bounces >= maxDepth)
			break;
		// << Sample BSDF at current vertex and compute reverse probability >>
		vec2 n = isect.n;
		vec2 tan = vec2(-n[1], n[0]);
		vec2 wi, wo = ray.dir;
		vec2 woLocal = vec2(dot(wo, tan), dot(wo, n));
		vec2 wiLocal = isect.obj->mat->sample(woLocal);
		wi = wiLocal[0] * tan + wiLocal[1] * n;
		pdfFwd = isect.obj->mat->pdf(woLocal, wiLocal);
		vec3 f = isect.obj->mat->bsdf(woLocal, wiLocal);
		beta *= f * abs(wiLocal[1]) / pdfFwd;
		pdfRev = isect.obj->mat->pdf(-wiLocal, -woLocal);
		ray.origin = ray.at(isect.tMax);
		ray.dir = wi;

		// << Compute reverse area density at preceding vertex >>
		prev.pdfRev = vertex.ConvertDensity(pdfRev, prev);
	}
	return bounces;
}

int GenerateLightSubpath(const Scene& scene, int maxDepth, Vertex* path) {
	if (maxDepth == 0)
		return 0;
	AreaLight* light = scene.arealight;
	vec2 nLight;
	double pdfPos, pdfDir;
	vec3 Le;
	Ray ray = light->sample_Le(pdfPos, pdfDir, nLight, Le);
	Vertex& l = path[0];
	l.normal = nLight;
	l.pos = ray.origin;
	l.pdfFwd = pdfPos;
	l.obj = light;
	l.type = VertexType::Light;

	// path[0] = l;
	vec3 beta = Le * abs(dot(nLight, ray.dir)) / (pdfPos * pdfDir);
	int nVertices = RandomWalk(scene, ray, beta, pdfDir,
		maxDepth - 1, TransportMode::Importance,
		path + 1);
	return nVertices + 1;

}

int GenerateCameraSubpath(const Scene& scene, Ray initRay, int maxDepth, Vertex* path) {
	if (maxDepth == 0)
		return 0;
	Ray ray = initRay;

	double pdfDir;
	Vertex& cam = path[0];
	cam.type = VertexType::Camera;
	cam.pos = ray.origin;
	cam.obj = nullptr;
	pdfDir = 1.0;
	vec3 color = 1.0;
	return RandomWalk(scene, ray, color, pdfDir,
		maxDepth - 1, TransportMode::Radiance,
		path + 1) + 1;
}


vec3 trace_path(Ray cameraRay, Scene& scene) {
	Vertex camera_vert[MAX_CAMERA_DEPTH + 2];
	Vertex light_vert[MAX_LIGHT_DEPTH + 1];
	Ray ray = cameraRay;
	int num_cam = GenerateCameraSubpath(scene, ray, MAX_CAMERA_DEPTH+2, camera_vert);
	int num_light = GenerateLightSubpath(scene, MAX_LIGHT_DEPTH+1, light_vert);
	vec3 color = 0.0;
	double ttl_iter = 0;
	for (int t = 2; t <= num_cam; t++) {
		for (int s = 0; s <= num_light; s++) {
			int depth = s + t - 2;
			if (depth < 0 || depth > TTL_MAX_DEPTH)
				continue;
			vec3 cpath = ConnectBDPT(scene, light_vert, camera_vert, s, t);
			color += cpath;
			++ttl_iter;
		}
	}
	return color;
}