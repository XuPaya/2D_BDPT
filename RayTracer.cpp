#include <iostream>
#include "adept_source.h"
#include <cmath>
#include <cstdlib>
#include <time.h> 
#include "vec.h"
#include "Ray.h"
#include "Scene.h"
#include "Intersect.h"
#include "bdpt.h"
#include "avec.h"
#include <ppl.h>

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif // !STB_IMAGE_IMPLEMENTATION

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#include <stb_image.h>
#include <stb_image_write.h>

using namespace std;

#define IMG_WIDTH 820
#define IMG_HEIGHT 461
//#define IMG_WIDTH 178
//#define IMG_HEIGHT 100
#define SAMPLE_PER_PIXEL 30
#define MAX_DEPTH 10


int diverged_rays = 0;



vec3 trace_ray(vec2 origin, Scene& scene) {
	// cout << origin << " " << theta << endl;
	double theta = uniform() * 2.0 * M_PI;
	vec2 dir(cos(theta), sin(theta));
	Ray ray = Ray(origin, dir);
	vec3 throughput = vec3(1.0);
	for (int i = 0; i < MAX_DEPTH; i++)
	{
		Intersect isec;
		scene.intersect(ray, isec);
		if (isec.tMax == 1e30) {
			diverged_rays++;
			// cout << "ray diverged" << endl;
			return vec3(0.0);
		}
		//cout << "ray: " << ray.origin << ", " << ray.dir << " tMax: " << isec.tMax << " mat: " << isec.mat << endl;

		vec2 n = isec.n / isec.n.length();
		vec2 tan = vec2(-n[1], n[0]);
		vec2 wiLocal = vec2(dot(ray.dir, tan),dot(ray.dir, n));
		vec2 woLocal = isec.obj->mat->sample(wiLocal);
		vec2 wo = woLocal[0] * tan + woLocal[1] * n;


		if (isec.obj->mat->type == matType::Light) {
			// cout << "hit light" << endl;
			return throughput * ((LightMaterial*)isec.obj->mat)->Le(wiLocal);
		}
		throughput *= isec.obj->mat->bsdf(wiLocal, woLocal) * abs(woLocal[1]) / isec.obj->mat->pdf(wiLocal, woLocal);

		ray.origin = ray.at(isec.tMax);
		ray.dir = wo / wo.length();
	}
	return vec3(0.0);
}

float image[IMG_HEIGHT][IMG_WIDTH][3];
unsigned char image_out[IMG_HEIGHT][IMG_WIDTH][3];
int main()
{
	//adept::Stack stack;
	//aVec2 v0(1.0, 2.0);
	//aVec2 v1(1.1, 3.5);
	//aVec2 v2;
	//stack.new_recording();
	//v2 = v0 * v1;
	//cout << v0 << endl;
	//cout << v1 << endl;
	//cout << v2 << endl;
	//v2.set_gradient(1.0, 1.0);
	//stack.compute_adjoint();
	//cout << v0.get_gradient() << endl;
	//cout << v1.get_gradient() << endl;
	//return 0;
	srand(2022);
	Scene scene;
	// world coordinate: [-1.78, 1.78] * [-1, 1]
	concurrency::parallel_for(0, IMG_WIDTH * IMG_HEIGHT, [&](int i)
	{
		if (i % IMG_WIDTH == 0)
			cout << i / IMG_WIDTH << endl;
		int xx = i % IMG_WIDTH;
		int yy = i / IMG_WIDTH;
		for (int sample = 0; sample < SAMPLE_PER_PIXEL; sample++) {
			double x = 2.0 * 1.78 * ((double)(xx + 0.5) / IMG_WIDTH - 0.5);
			double y = 2.0 * ((double)(yy + 0.5) / IMG_HEIGHT - 0.5);
			double theta = (sample + uniform()) / SAMPLE_PER_PIXEL * (2.0 * M_PI);
			// double theta = uniform() * (2.0 * M_PI);
			vec2 origin(x, y);
			vec2 dir(cos(theta), sin(theta));
			scene.camera_pos = origin;
			vec3 rad = trace_path(Ray(origin, dir), scene);
			// return 0;
			//if (rad.length() > 0)
			//{
			//	cout << "x, y: " << x << ", " << y << endl;
			//	cout << rad << endl;
			//}
			for (int c = 0; c < 3; c++) {
				image[yy][xx][c] += (float)rad[c] / SAMPLE_PER_PIXEL;
			}
		}
		for (int c = 0; c < 3; c++) {
			image[yy][xx][c] = clamp(pow(image[yy][xx][c], 1.0 / 2.2), 0.0, 1.0);
		}

		for (int c = 0; c < 3; c++) {
			image_out[IMG_HEIGHT - yy - 1][xx][c] = (uint8_t)(image[yy][xx][c] * 255);
		}
		//cout << diverged_rays << endl;
		//diverged_rays = 0;
	});

	//for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
	//		if (i % IMG_WIDTH == 0)
	//			cout << i / IMG_WIDTH << endl;
	//		int xx = i % IMG_WIDTH;
	//		int yy = i / IMG_WIDTH;
	//		for (int sample = 0; sample < SAMPLE_PER_PIXEL; sample++) {
	//			double x = 2.0 * 1.78 * ((double)(xx + 0.5) / IMG_WIDTH - 0.5);
	//			double y = 2.0 * ((double)(yy + 0.5) / IMG_HEIGHT - 0.5);
	//			double theta = (sample + uniform()) / SAMPLE_PER_PIXEL * (2.0 * M_PI);
	//			// double theta = uniform() * (2.0 * M_PI);
	//			vec2 origin(x, y);
	//			vec2 dir(cos(theta), sin(theta));
	//			scene.camera_pos = origin;
	//			vec3 rad = trace_path(Ray(origin, dir), scene);
	//			// return 0;
	//			//if (rad.length() > 0)
	//			//{
	//			//	cout << "x, y: " << x << ", " << y << endl;
	//			//	cout << rad << endl;
	//			//}
	//			for (int c = 0; c < 3; c++) {
	//				image[yy][xx][c] += (float)rad[c] / SAMPLE_PER_PIXEL;
	//			}
	//		}
	//		for (int c = 0; c < 3; c++) {
	//			image[yy][xx][c] = clamp(pow(image[yy][xx][c], 1.0 / 2.2), 0.0, 1.0);
	//		}

	//		for (int c = 0; c < 3; c++) {
	//			image_out[IMG_HEIGHT - yy - 1][xx][c] = (uint8_t)(image[yy][xx][c] * 255);
	//		}
	//		//cout << diverged_rays << endl;
	//		//diverged_rays = 0;
	//};
	//
	stbi_write_bmp("output.bmp", IMG_WIDTH, IMG_HEIGHT, 3, image_out);
	return 0;
}
