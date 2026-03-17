#pragma once
#include <cmath>
#include <array>
#define CIRCLE_TABLE_RESOLUTION 360
#define VEC3_SEG_ORDER 6

#define degreeToArc 0.01745329251f

__attribute__((always_inline)) inline
float recipsf2(float input) {
    float result, temp;
    asm(
        "recip0.s %0, %2\n"
        "const.s %1, 1\n"
        "msub.s %1, %2, %0\n"
        "madd.s %0, %0, %1\n"
        "const.s %1, 1\n"
        "msub.s %1, %2, %0\n"
        "maddn.s %0, %0, %1\n"
        :"=&f"(result),"=&f"(temp):"f"(input)
    );
    return result;
}

#define DIV(a, b) (a)*recipsf2(b)

struct Vec3 {
	float x{ 0 };
	float y{ 0 };
	float z{ 0 };
	Vec3(float x_in = 0, float y_in = 0, float z_in = 0) { x = x_in; y = y_in; z = z_in; }
	Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z;  return *this; }
	Vec3& operator-=(const Vec3& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
	Vec3& operator*=(const float m) { x *= m; y *= m; z *= m; return *this; }
	Vec3& operator*=(const Vec3& b) { x *= b.x; y *= b.y; z *= b.z; return *this; }
	Vec3& operator/=(const float m) { float im = DIV(1, m); return operator*=(im); }
	Vec3 operator-()const { return Vec3(*this).operator*=(-1); }
};
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return Vec3(a).operator+=(b); }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return Vec3(a).operator-=(b); }
inline Vec3 operator/(const Vec3& a, const float& b) { return Vec3(a).operator/=(b); }
inline Vec3 operator*(const Vec3& a, const float& b) { return Vec3(a).operator*=(b); }
inline Vec3 operator*(const Vec3& a, const Vec3& b) { return Vec3(a).operator*=(b); }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float squareLength(const Vec3& a) { return dot(a, a); }
inline float length(const Vec3& a) { return sqrtf(dot(a, a)); }
inline bool operator==(const Vec3& a, const Vec3& b) { return squareLength(a - b) < 1E-14f; }
inline Vec3 normalize(const Vec3& a) { return a / std::max(1.0E-7f, length(a)); }

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline float det(const Vec3& a, const Vec3& b, const Vec3& c){return dot(a, cross(b, c));}

class CircleTable{
private:
	static std::array<float, CIRCLE_TABLE_RESOLUTION> sinTable;
	static std::array<float, CIRCLE_TABLE_RESOLUTION> cosTable;
	static float minDegree;
public:
	static void init();
	static float fastSin(int index);
	static float fastCos(int index);
	static float fastSin(float degree){ return fastSin(int(degree / minDegree)); }
	static float fastCos(float degree){ return fastCos(int(degree / minDegree)); }
	static float getMinDegree(){return minDegree;}
};
