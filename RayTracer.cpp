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
#include <fstream>


#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif // !STB_IMAGE_IMPLEMENTATION

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#include <stb_image.h>
#include <stb_image_write.h>

using namespace std;

//#define IMG_WIDTH 820
//#define IMG_HEIGHT 461
#define IMG_WIDTH 89
#define IMG_HEIGHT 50
#define SAMPLE_PER_PIXEL 32
#define MAX_DEPTH 10

int counter = 0;
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

double image[IMG_HEIGHT][IMG_WIDTH][3];
double image0[IMG_HEIGHT][IMG_WIDTH][3];
unsigned char image_out[IMG_HEIGHT][IMG_WIDTH][3];
unsigned char image_out0[IMG_HEIGHT][IMG_WIDTH][3];
double diff[IMG_HEIGHT][IMG_WIDTH][3];
int main()
{
	for (int i = 0; i < IMG_HEIGHT; i++) {
		for (int j = 0; j < IMG_WIDTH; j++) {
			for (int k = 0; k < 3; k++) {
				image[i][j][k] = 0;
				image0[i][j][k] = 0;
				diff[i][j][k] = 0;
			}
		}
	}
	
	Scene scene;

	srand(2021);
	//srand(time(NULL));
	//adept::Stack stack0;
	//aVertex xMinus;
	//aVertex x;
	//aVertex xPlus;
	//xMinus.pos = vec2(-1.78, 1.0);
	//xPlus.pos = vec2(-1.78, -1.0);
	//x.obj = xMinus.obj = xPlus.obj = scene.objs[0];
	//x.normal = vec2(-1, 0);
	//xMinus.normal = vec2(1, 0);
	//xMinus.obj = scene.arealight;
	//adept::adouble theta = 0.0;
	//scene.scene_theta = &theta;
	//vec2 V;
	//double divV;
	//double c0 = 0, c1 = 0;
	//double grad = 0.0;
	////for (int i = 0; i < 1000; i++) {
	////	x.pos = vec2(0.5, 0.4384);
	////	warpFieldUnidir(stack0, scene, x, xMinus, V, divV);
	////	double x0 = V[1];
	////	double y0 = divV;
	////	//cout << divV << ", " << V[1] << endl;

	////	//x.pos = vec2(1.78, 0.5189);
	////	//warpField(stack0, scene, xMinus, x, xPlus, V, divV);
	////	//double x1 = V[1];
	////	//double y1 = divV;
	////	//cout << divV << ", " << V[1] << endl;
	////	c0 += x0;
	////	//c1 += (y0 + y1) / 2;
	////	grad += divV;
	////	cout << c0 / 1000 << ", " << c1 / 1000 << ", " << grad / (i + 1.0) << endl;
	////}

	////cout << c0 / 1000 << ", " << c1 / 1000 << endl;
	//int iter = 1000;
	//ofstream warp_out;
	//warp_out.open("warp_out_unweighted.txt");
	//double cache = 0.0;
	//for (int i = 0; i < iter; i++){
	//	x.pos = vec2(1.78, 1.0 - (i+0.5) * 2.0 / iter);
	//	warpField(stack0, scene, xMinus, x, xPlus, V, divV);
	//	warp_out << x.pos[1] << " " << V[1] << " " << divV << endl;
	//	if (i % 50 == 0)
	//		cout << i / 1000.0 << endl;
	//	// cout << divV << ", " << (V[1] - cache) / 2.0 * iter << endl;
	//	// cache = V[1];
	//}
	//warp_out.close();

	//return 0;
	// world coordinate: [-1.78, 1.78] * [-1, 1]

	double left_bdry = 0.0;
	double right_bdry = 1.78;
	double lower_bdry = 0.0;
	double upper_bdry = 1.0;

	left_bdry = -0.53;
	right_bdry = 0.53;
	lower_bdry = 0.3;
	upper_bdry = 0.9;
	cout << "image 0:" << endl;
	concurrency::parallel_for(0, IMG_WIDTH * IMG_HEIGHT, [&](int i)
	{
		adept::Stack stack;
		adept::adouble theta = 0;
		scene.scene_theta = &theta;
		if (i % IMG_WIDTH == 0) {
			cout << ++counter << endl;
		}
		int xx = i % IMG_WIDTH;
		int yy = i / IMG_WIDTH;
		for (int sample = 0; sample < SAMPLE_PER_PIXEL; sample++) {
			double x = (right_bdry - left_bdry) * ((double)(xx + uniform()) / IMG_WIDTH) + left_bdry;
			double y = (upper_bdry - lower_bdry) * ((double)(yy + uniform()) / IMG_HEIGHT) + lower_bdry;
			double theta = (sample + uniform()) / SAMPLE_PER_PIXEL * (2.0 * M_PI);
			// double theta = uniform() * (2.0 * M_PI);
			vec2 origin(x, y);
			vec2 dir(cos(theta), sin(theta));
			scene.camera_pos = origin;
			vec3 dLdtheta;
			vec3 rad = trace_path(stack, Ray(origin, dir), scene, dLdtheta);
			for (int c = 0; c < 3; c++) {
				image[yy][xx][c] += rad[c] / (SAMPLE_PER_PIXEL);
				diff[yy][xx][c] += dLdtheta[c] / SAMPLE_PER_PIXEL;
			}
		}
		for (int c = 0; c < 3; c++) {
			image_out[IMG_HEIGHT - yy - 1][xx][c] = (uint8_t)(clamp(pow(image[yy][xx][c], 1.0 / 2.2), 0.0, 1.0) * 255);
		}
	});
	//
	//srand(2021);
	//counter = 0;
	//cout << "image 1:" << endl;
	//concurrency::parallel_for(0, IMG_WIDTH * IMG_HEIGHT, [&](int i)
	//{
	//	adept::Stack stack;
	//	adept::adouble theta = 2e-3;
	//	scene.scene_theta = &theta;
	//	if (i % IMG_WIDTH == 0) {
	//		cout << ++counter << endl;
	//	}
	//	int xx = i % IMG_WIDTH;
	//	int yy = i / IMG_WIDTH;
	//	for (int sample = 0; sample < SAMPLE_PER_PIXEL; sample++) {
	//		double x = (right_bdry - left_bdry) * ((double)(xx + uniform()) / IMG_WIDTH) + left_bdry;
	//		double y = (upper_bdry - lower_bdry) * ((double)(yy + uniform()) / IMG_HEIGHT) + lower_bdry;
	//		double theta = (sample + uniform()) / SAMPLE_PER_PIXEL * (2.0 * M_PI);
	//		// double theta = uniform() * (2.0 * M_PI);
	//		vec2 origin(x, y);
	//		vec2 dir(cos(theta), sin(theta));
	//		scene.camera_pos = origin;
	//		vec3 dLdtheta;
	//		vec3 rad = trace_path(stack, Ray(origin, dir), scene, dLdtheta);
	//		for (int c = 0; c < 3; c++) {
	//			image0[yy][xx][c] += rad[c] / (SAMPLE_PER_PIXEL);
	//		}
	//	}
	//	for (int c = 0; c < 3; c++) {
	//		image_out0[IMG_HEIGHT - yy - 1][xx][c] = (uint8_t)(clamp(pow(image0[yy][xx][c], 1.0 / 2.2), 0.0, 1.0) * 255);
	//	}
	//});
	ofstream file;
	file.open("warpfield_boundary.txt");
	for (int y = 0; y < IMG_HEIGHT; y++) {
		for (int x = 0; x < IMG_WIDTH; x++) {
			for (int c = 0; c < 3; c++) {
				// diff[y][x][c] = (image0[y][x][c] - image[y][x][c]) / (double)4e-3;
				file << diff[y][x][c] << " ";
			}
			file << endl;
		}
	}
	file.close();
	stbi_write_bmp("output.bmp", IMG_WIDTH, IMG_HEIGHT, 3, image_out);
	// stbi_write_bmp("output0.bmp", IMG_WIDTH, IMG_HEIGHT, 3, image_out0);
	return 0;
}
