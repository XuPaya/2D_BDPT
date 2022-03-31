#pragma once
#include <iostream>
#include <cstdlib>

using namespace std;
#define M_PI 3.14159265358979323846

double min(double a, double b) {
	return (a < b ? a : b);
}

double max(double a, double b) {
	return (a < b ? b : a);
}


double uniform() {
	return (double)rand() / ((double)RAND_MAX + 1);
}

double clamp(double x, double xmin, double xmax) {
	return max(xmin, min(x, xmax));
}

double sign(double x) {
	if (x > 0.0) return 1.0;
	if (x == 0.0) return 0.0;
	return -1.0;
}
