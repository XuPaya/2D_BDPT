#pragma once
#include "vec.h"
class mat2 { 
// 0 1
// 2 3
public:
	double e[4];
	mat2(): e{0, 0, 0, 0} {}
	mat2(double e11, double e12, double e21, double e22): e{ e11, e12, e21, e22 } {}
	mat2 operator-() {
		return mat2(-e[0], -e[1], -e[2], -e[3]);
	}
	const double* operator[] (int i) const {
		return e + i * 2;
	}
	double* operator[] (int i) {
		return e + i * 2;
	}

	mat2& operator+=(const mat2& m) {
		for (int i = 0; i < 4; i++)
			e[i] += m.e[i];
		return *this;
	}

	mat2& operator+=(const double t) {
		for (int i = 0; i < 4; i++)
			e[i] += t;
		return *this;
	}

	mat2& operator*=(const double t) {
		for (int i = 0; i < 4; i++)
			e[i] *= t;
		return *this;
	}

	mat2& operator/=(const double t) {
		return *this *= (1.0 / t);
	}

	mat2 inverse() {
		double inv_det = 1.0 / (e[0] * e[3] - e[1] * e[2]);
		return mat2(e[3] * inv_det, -e[1] * inv_det, -e[2] * inv_det, e[0] * inv_det);
	}

};

inline mat2 operator*(const mat2& u, const mat2& v) {
	return mat2(u.e[0] * v.e[0] + u.e[1] * v.e[2], u.e[0] * v.e[1] + u.e[1] * v.e[3],
				u.e[2] * v.e[0] + u.e[3] * v.e[2], u.e[2] * v.e[1] + u.e[3] * v.e[3]);
}

inline vec2 operator*(const mat2& u, const vec2& v) {
	return vec2(u.e[0] * v.e[0] + u.e[1] * v.e[1], u.e[2] * v.e[0] + u.e[3] * v.e[1]);
}

inline vec2 operator*(const vec2& v, const mat2& u) {
	return vec2(u.e[0] * v.e[0] + u.e[2] * v.e[1], u.e[1] * v.e[0] + u.e[3] * v.e[1]);
}

inline std::ostream& operator<<(std::ostream& out, const mat2& m) {
	return out << m.e[0] << ' ' << m.e[1] << ' ' << m.e[2] << ' ' << m.e[3];
}
