#pragma once
#include "Scene.h"
#include "Vertex.h"
#include <vector>
#include "adept.h"
#include "mat.h"

#define ALLOW_AD 1
#define MAX_CAMERA_DEPTH 3
#define MAX_LIGHT_DEPTH 3
#define TTL_MAX_DEPTH 6
#define NUM_AUX_SAMPLES 64
using namespace adept;

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

double visibility_test(Scene& scene, aVertex& v0, aVertex& v1) {
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

double G(Scene& scene, aVertex& v0, aVertex& v1) {
	vec2 d = v0.p() - v1.p();
	double g = 1.0 / d.length();
	d = d.normalized();
	if (v0.type != VertexType::Camera)
		g *= abs(dot(v0.n(), d));
	if (v1.type != VertexType::Camera)
		g *= abs(dot(v1.n(), d));
	return g;
}

adouble ad_G(Scene& scene, aVec2& x0, aVec2& x1, vec2& n0, vec2& n1) {
	aVec2 d = x0 - x1;
	adept::adouble g = 1.0 / d.length();
	d = d.normalized();
	if (n0.length() > 0.5)
		g *= adept::abs(d.e[0] * n0[0] + d.e[1] * n0[1]);
	if (n1.length() > 0.5)
		g *= adept::abs(d.e[0] * n1[0] + d.e[1] * n1[1]);
	return g;
}

double MISWeight(Scene& scene, aVertex* lightVertices,
	aVertex* cameraVertices, aVertex& sampled, int s, int t) {
	if (s + t == 2)
		return 1;
	double sumRi = 0.0;
	// << Define helper function remap0 that deals with Dirac delta functions >>
	auto remap0 = [](double f) -> double { return f != 0 ? f : 1; };

	// << Temporarily update vertex properties for current strategy>>
	//	<<Look up connection vertices and their predecessors >>
	aVertex* qs = s > 0 ? &lightVertices[s - 1] : nullptr,
		* pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
		* qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
		* ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

	// << Update sampled vertex for or strategy >>
	ScopedAssignment<aVertex> a1;
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

void sampleNormal(double sigma, double mu, double& x, double& pdf) {
	x = sigma * sqrt(-2.0 * log(1 - uniform())) * cos(2 * M_PI * uniform()) + mu;
	pdf = 1.0 / (sigma * sqrt(2.0 * M_PI)) * exp(-0.5 * pow((x - mu) / sigma, 2));
}

void convVUnidir(adept::Stack& stack, Scene& scene, aVertex& xAdj, aVertex& x, vec2& dxdt, double& B) {
	aVec2 origin = xAdj.p();
	aVec2 x_aux = x.p();
	aVec2 dir = (x_aux - origin).normalized();
	adept::adouble ang = acos(dir[0]);
	if (dir[1] < 0.0) ang = 2 * M_PI - ang;
	double jac[4] = {};

	stack.new_recording();
	aVec2 ad_dir = aVec2(adept::cos(ang), adept::sin(ang));
	aRay ray(origin, ad_dir);
	diff_Intersect isec_scene;
	diff_Intersect isec_x;
	// isec.stack = &stack;
	scene.diff_intersect(ray, isec_scene);
	aVec2 xHalf = isec_scene.pos;
	stack.independent(*(scene.scene_theta));
	stack.dependent(xHalf.e, 2);
	stack.jacobian(jac);
	vec2 dhalf_dt(jac[0], jac[1]);


	aVec2 xHalf_ind = (xHalf).detached();
	stack.new_recording();
	aRay ray_x(origin, (xHalf_ind - origin).normalized());
	x.obj->diff_intersect(ray_x, isec_x, *(scene.scene_theta));
	aVec2& xi = isec_x.pos;

	stack.independent(xHalf_ind.e, 2);
	stack.dependent(xi.e, 2);
	stack.jacobian(jac);
	mat2 dx_dhalf(jac[0], jac[2], jac[1], jac[3]);
	//if (isec_scene.obj != scene.arealight) {
	//	cout << "hit something else" << endl;
	//	cout << isec_x.tMax << ", " << dx_dhalf << endl;
	//	cout <<  (xHalf_ind) << ", " << (x_aux) << endl;
	//}
	// 
	dxdt = dx_dhalf * dhalf_dt;
	B = isec_scene.B;
}

double warpFieldUnidir(adept::Stack& stack, Scene& scene, aVertex& xAdj, aVertex& x, vec2& V, double& divV) {
	vec2 x_pos = x.p();
	vec2 n = x.n();
	vec2 tan = vec2(-n[1], n[0]);
	double norm_var, aux_pdf;
	double Z = 0.0;
	vec2 dZ = vec2(0.0);
	double w_arr[NUM_AUX_SAMPLES];
	vec2 dw_arr[NUM_AUX_SAMPLES];
	vec2 V_arr[NUM_AUX_SAMPLES];
	double sigma = 0.3;
	aVertex x_aux;
	x_aux.obj = x.obj;
	double F = 0.0;
	for (int i = 0; i < NUM_AUX_SAMPLES; i++) {
		sampleNormal(sigma, 0.0, norm_var, aux_pdf);
		if (i == 0) {
			norm_var = 0;
			aux_pdf = 1.0 / (sigma * sqrt(2.0 * M_PI));
		}
		x_aux.pos = x_pos + tan * norm_var; // this should be replaced by surface sampling
		double B;
		convVUnidir(stack, scene, xAdj, x_aux, V_arr[i], B);
		aVec2 ad_x_pos = x_pos;
		stack.new_recording();
		adept::adouble dist = adept::sqrt(adept::pow((ad_x_pos.e[0] - x_aux.pos[0]), 2) + adept::pow((ad_x_pos.e[1] - x_aux.pos[1]), 2));
		adept::adouble D = 1.0 / (sigma * sqrt(2.0 * M_PI)) * (1.0 - exp(-0.5 * adept::pow(dist / sigma, 2)));
		adept::adouble w = 1.0 / (D + B);

		double jac[2];

		stack.independent(ad_x_pos.e, 2);
		stack.dependent(w);
		stack.jacobian(jac);
		dw_arr[i] = vec2(jac[0], jac[1]) / aux_pdf;

		w_arr[i] = w.value() / aux_pdf;
		// cout << "var: " << norm_var << ", dwMinus: " << dwMinus_arr[i] << ", wMinus: " << wMinus_arr[i] << endl;

		//double temp1 = wPlus * wPlus / (wMinus + wPlus);
		F += w_arr[i];
		//G += temp1;
		Z += w_arr[i];
		dZ += dw_arr[i];
	}
	//cout << "F: " << F << ", G: " << G << endl;
	//cout << "Dminus: " << dMinus / ZMinus << ", Dplus: " << dPlus / ZPlus << endl;
	vec2 V_holder(0);
	double dwV = 0, wVd = 0;
	for (int i = 0; i < NUM_AUX_SAMPLES; i++) {
		V_holder += w_arr[i] * V_arr[i];
		dwV += dot(dw_arr[i], V_arr[i]);
		wVd += w_arr[i] * dot(V_arr[i], dZ);
	}
	V = V_holder / Z;
	divV = dwV / Z - wVd / (Z * Z);
	return F;
	// cout << "term1: " << dwVMinus / ZMinus << ", term2: " << wVdMinus / (ZMinus * ZMinus) << endl;
}

void warpField(adept::Stack& stack, Scene& scene, aVertex& xMinus, aVertex& x, aVertex& xPlus, vec2& V, double& divV) {
	vec2 VMinus = vec2(0.0), VPlus = vec2(0.0);
	double divVMinus = 0.0, divVPlus = 0.0;
	double F = warpFieldUnidir(stack, scene, xMinus, x, VMinus, divVMinus);
	double G = warpFieldUnidir(stack, scene, xPlus, x, VPlus, divVPlus);
	V = F / (F + G) * VMinus + G / (F + G) * VPlus;
	divV = F / (F + G) * divVMinus + G / (F + G) * divVPlus;
}

vec3 evalPath(adept::Stack& stack, Scene& scene, vector<aVertex*> path, vec3& dLdtheta) { // Path Integral, all gradient are calculated in this function
	//cout << "Path from light to camera: " << endl;
	//for (int i = 0; i < path.size(); i++)
	//{
	//	cout << path[i]->p() << endl;
	//}
	//cout << "end" << endl << endl;
#if ALLOW_AD
	stack.new_recording();
#endif // ALLOW_AD

	double pdfPath = path[0]->pdfFwd;
	aVec3 L = path[0]->Le(*path[1]) * ad_G(scene, path[0]->ad_pos, path[1]->ad_pos, path[0]->n(), path[1]->n());

	for (int i = 1; i < path.size() - 1; i++) {
		pdfPath *= path[i]->pdfFwd;
		L *= path[i]->f(*path[i - 1], *path[i + 1]) * ad_G(scene, path[i]->ad_pos, path[i + 1]->ad_pos, path[i]->n(), path[i + 1]->n());
	}

	L *= path[path.size() - 1]->We(*path[path.size() - 2]) / pdfPath;

#if ALLOW_AD
	vec3 interior = vec3(0.0);
	vec3 boundary = vec3(0.0);
	stack.dependent(L.e, 3);
	double jac[6];
	for (int i = 0; i < path.size(); i++) {
		stack.clear_independents();
		stack.independent(path[i]->ad_pos.e, 2);
		stack.jacobian(jac);
		interior += vec3(jac[0] * path[i]->buffer.dxdtheta[0] + jac[3] * path[i]->buffer.dxdtheta[1],
			jac[1] * path[i]->buffer.dxdtheta[0] + jac[4] * path[i]->buffer.dxdtheta[1],
			jac[2] * path[i]->buffer.dxdtheta[0] + jac[5] * path[i]->buffer.dxdtheta[1]);
		vec2 V = vec2(0.0);
		double divV = 0.0;
		if (i == 0) {
			warpFieldUnidir(stack, scene, *path[1], *path[0], V, divV);
			//if (path.size() < 3)
			//	cout << divV << endl;
		}
		else if (i != path.size() - 1) {
			warpField(stack, scene, *path[i - 1], *path[i], *path[i + 1], V, divV);
		}
		//if (path.size() < 3 && divV > 0)
		//	cout << V << ", " << divV << ", " << path[0]->p() << ", " << path[1]->p() << endl;
		boundary += (vec3(jac[0] * V[0] + jac[3] * V[1],
			jac[1] * V[0] + jac[4] * V[1],
			jac[2] * V[0] + jac[5] * V[1]) + L.detached() * divV);
	}
	dLdtheta = (boundary + interior);
#endif // ALLOW_AD
	return L.detached();
}

vec3 ConnectBDPT(adept::Stack& stack, Scene& scene, aVertex* lightVertices,
	aVertex* cameraVertices, int s, int t, vec3& dLdtheta) {
	vec3 L(0.f);
	//adept::Stack stack;
	//stack.new_recording();
	// << Perform connection and write contribution to L >>
	aVertex sampled;
	vector<aVertex*> path;
	if (s == 0) {
		// << Interpret the camera subpath as a complete path >>
		const aVertex & pt = cameraVertices[t - 1];
		if (pt.obj == scene.arealight) {
			for (int i = t - 1; i >= 0; i--) {
				path.push_back(&cameraVertices[i]);
			}
		}

	} // camera is not connectible, so ignore t == 1
	else if (s == 1) {
		// << Sample a point on a light and connect it to the camera subpath >>
		aVertex & pt = cameraVertices[t - 1], & ptMinus = cameraVertices[t - 2];
		double vis = 1.0;
		vec2 wi; 
		double pdf;
		AreaLight* light = scene.arealight;
		vec2 u;
		vec3 lightWeight = light->sample_Li(pt.p(), u, pdf, wi);
		if (pdf > 0) {
			sampled.normal = vec2(1.0, 0.0);
			sampled.pos = u;
			sampled.ad_pos = sampled.pos;
			sampled.obj = scene.arealight;
			sampled.type = VertexType::Light;
			sampled.pdfFwd = sampled.PdfLightOrigin(scene, pt);
			vis = visibility_test(scene, sampled, pt);
			if (vis == 0.0) {
				return vec3(0.0);
			}
			path.push_back(&sampled);
			for (int i = t - 1; i >= 0; i--) {
				path.push_back(&cameraVertices[i]);
			}
		}
	}
	else {
		// << Handle all other bidirectional connection cases >>
		aVertex & qs = lightVertices[s - 1], &pt = cameraVertices[t - 1], &qsMinus = lightVertices[s - 2], &ptMinus = cameraVertices[t - 2];
		double vis = visibility_test(scene, qs, pt);
		if (vis == 0.0) {
			return vec3(0.0);
		}
		for (int i = 0; i < s; i++) {
			path.push_back(&lightVertices[i]);
		}
		for (int i = t - 1; i >= 0; i--) {
			path.push_back(&cameraVertices[i]);
		}
	}
	if (path.size() != 0) {
		// cout << "s = " << s << ", t = " << t << endl;
		L = evalPath(stack, scene, path, dLdtheta);
	}
	double misWeight = MISWeight(scene, lightVertices, cameraVertices, sampled, s, t);
	L *= misWeight;
	dLdtheta *= misWeight;
	return L;
}

int RandomWalk(adept::Stack& stack, Scene& scene, Ray ray, vec3 beta, double pdf, int maxDepth,
	TransportMode mode, aVertex* path) {
	if (maxDepth == 0)
		return 0;
	int bounces = 0;
	double pdfFwd = pdf, pdfRev = 0;

	while (true) {
		if (beta.length() < 1e-10)
			break;
		double t = 1e30;
		vec2 n = vec2(0.0);
#if ALLOW_AD
		aVec2 ad_ori = ray.origin;
		aVec2 ad_dir = ray.dir;
		aRay aray(ad_ori, ad_dir);
		diff_Intersect isect;
		stack.new_recording();

		scene.diff_intersect(aray, isect);

		aVec2 x = aVec2(aray.origin.e[0] + aray.dir.e[0] * isect.tMax, aray.origin.e[1] + aray.dir.e[1] * isect.tMax);
		stack.dependent(x.e, 2);
		stack.independent(*(scene.scene_theta));
		double jac[2];
		stack.jacobian(jac);
		t = isect.tMax.value();
		n = isect.n.detached();
#else
		Intersect isect;
		scene.intersect(ray, isect);
		t = isect.tMax;
		n = isect.n;
#endif

		
		aVertex& vertex = path[bounces], & prev = path[bounces - 1];

		if (isect.tMax == 1e30) {
			break;
		}

#if ALLOW_AD
		vertex.buffer.dxdtheta = vec2(jac[0], jac[1]);
#endif
		vertex.pos = ray.origin + ray.dir * t;
		vertex.ad_pos = vertex.pos;
		vertex.normal = n;
		vertex.obj = isect.obj;
		vertex.type = VertexType::Surface;
		vertex.pdfFwd = prev.ConvertDensity(pdfFwd, vertex);

		if (++bounces >= maxDepth)
			break;
		// << Sample BSDF at current vertex and compute reverse probability >>
		vec2 tan = vec2(-n[1], n[0]);
		vec2 wi, wo = ray.dir;
		vec2 woLocal = vec2(dot(wo, tan), dot(wo, n));
		vec2 wiLocal = isect.obj->mat->sample(woLocal);
		wi = wiLocal[0] * tan + wiLocal[1] * n;
		pdfFwd = isect.obj->mat->pdf(woLocal, wiLocal);
		vec3 f = isect.obj->mat->bsdf(woLocal, wiLocal);
		beta *= f * abs(wiLocal[1]) / pdfFwd;
		pdfRev = isect.obj->mat->pdf(-wiLocal, -woLocal);
		ray.origin = ray.origin + ray.dir * t;
		ray.dir = wi;

		// << Compute reverse area density at preceding vertex >>
		prev.pdfRev = vertex.ConvertDensity(pdfRev, prev);
	}
	return bounces;
}

int GenerateLightSubpath(adept::Stack& stack, Scene& scene, int maxDepth, aVertex* path) {
	if (maxDepth == 0)
		return 0;
	AreaLight* light = scene.arealight;
	vec2 nLight;
	double pdfPos, pdfDir;
	vec3 Le;
	Ray ray = light->sample_Le(pdfPos, pdfDir, nLight, Le);
	aVertex& l = path[0];
	l.normal = nLight;
	l.pos = ray.origin;
	l.ad_pos = l.pos;
	l.pdfFwd = pdfPos;
	l.obj = light;
	l.type = VertexType::Light;

	// path[0] = l;
	vec3 beta = Le * abs(dot(nLight, ray.dir)) / (pdfPos * pdfDir);
	int nVertices = RandomWalk(stack, scene, ray, beta, pdfDir,
		maxDepth - 1, TransportMode::Importance,
		path + 1);
	return nVertices + 1;

}

int GenerateCameraSubpath(adept::Stack& stack, Scene& scene, Ray initRay, int maxDepth, aVertex* path) {
	if (maxDepth == 0)
		return 0;

	double pdfDir;
	aVertex& cam = path[0];
	cam.type = VertexType::Camera;
	cam.pos = initRay.origin;
	cam.ad_pos = cam.pos;
	cam.obj = nullptr;
	cam.pdfFwd = 1.0;
	pdfDir = 1.0;
	vec3 color(1.0);
	return RandomWalk(stack, scene, initRay, color, pdfDir,
		maxDepth - 1, TransportMode::Radiance,
		path + 1) + 1;
}


vec3 trace_path(adept::Stack& stack, Ray cameraRay, Scene& scene, vec3& dLdtheta) {
	aVertex camera_vert[MAX_CAMERA_DEPTH + 2];
	aVertex light_vert[MAX_LIGHT_DEPTH + 1];
	int num_cam = GenerateCameraSubpath(stack, scene, cameraRay, MAX_CAMERA_DEPTH+2, camera_vert);
	int num_light = GenerateLightSubpath(stack, scene, MAX_LIGHT_DEPTH+1, light_vert);
	vec3 color = vec3(0.0);
	vec3 accum_dLdtheta = vec3(0.0);
	for (int t = 2; t <= num_cam; t++) {
		for (int s = 0; s <= num_light; s++) {
			int depth = s + t - 2;
			if (depth < 0 || depth > TTL_MAX_DEPTH)
				continue;
			vec3 holder = vec3(0.0);
			vec3 cpath = ConnectBDPT(stack, scene, light_vert, camera_vert, s, t, holder);
			accum_dLdtheta += holder;
			color += cpath;
		}
	}
	dLdtheta = accum_dLdtheta;
	return color;
}