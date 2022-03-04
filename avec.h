#pragma once

#include <cmath>
#include <iostream>
#include "adept_arrays.h"
#include "vec.h"

class aVec2 {
public:
    aVec2() : e{ 0,0 } {}
    aVec2(double x) : e{ x, x } {}
    aVec2(double e0, double e1) : e{ e0, e1 } {}
    aVec2(adept::adouble e0, adept::adouble e1) : e{ e0, e1 } {}

    adept::adouble x() const { return e[0]; }
    adept::adouble y() const { return e[1]; }

    aVec2 operator-() const { return aVec2(-e[0], -e[1]); }
    adept::adouble operator[](int i) const { return e[i]; }
    adept::adouble& operator[](int i) { return e[i]; }

    aVec2& operator+=(const aVec2& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        return *this;
    }

    aVec2& operator*=(const double t) {
        e[0] *= t;
        e[1] *= t;
        return *this;
    }

    aVec2& operator*=(const aVec2& v) {
        e[0] *= v[0];
        e[1] *= v[1];
        return *this;
    }

    aVec2& operator/=(const double t) {
        return *this *= 1 / t;
    }

    aVec2 normalized() {
        adept::adouble invL = adept::adouble(1.0) / this->length();
        return aVec2(e[0] * invL, e[1] * invL);
    }

    adept::adouble length() const {
        return adept::sqrt(length_squared());
    }

    adept::adouble length_squared() const {
        return e[0] * e[0] + e[1] * e[1];
    }

    void set_gradient(double d0, double d1) {
        e[0].set_gradient(d0);
        e[1].set_gradient(d1);
    }
    
    vec2 get_gradient() {
        dL_de[0] = e[0].get_gradient();
        dL_de[1] = e[1].get_gradient();
        return vec2(dL_de[0], dL_de[1]);
    }

public:
    adept::adouble e[2];
    double dL_de[2] = {0, 0};
};


inline aVec2 operator+(const aVec2& u, const aVec2& v) {
    return aVec2(u.e[0] + v.e[0], u.e[1] + v.e[1]);
}

inline aVec2 operator-(const aVec2& u, const aVec2& v) {
    return aVec2(u.e[0] - v.e[0], u.e[1] - v.e[1]);
}

inline aVec2 operator*(const aVec2& u, const aVec2& v) {
    return aVec2(u.e[0] * v.e[0], u.e[1] * v.e[1]);
}

inline aVec2 operator*(adept::adouble t, const aVec2& v) {
    return aVec2(t * v.e[0], t * v.e[1]);
}

inline aVec2 operator*(const aVec2& v, adept::adouble t) {
    return t * v;
}
//
//inline aVec2 operator/(aVec2 v, double t) {
//    return (1/t) * v;
//}

inline aVec2 operator/(aVec2 v, adept::adouble t) {
    return aVec2(v.e[0] / t, v.e[1] / t);
}

inline adept::adouble dot(const aVec2& u, const aVec2& v) {
    return u.e[0] * v.e[0]
        + u.e[1] * v.e[1];
}

inline aVec2 unit_vector(aVec2 v) {
    return v / v.length();
}

inline std::ostream& operator<<(std::ostream& out, const aVec2& v) {
    return out << v.e[0] << ' ' << v.e[1];
}
