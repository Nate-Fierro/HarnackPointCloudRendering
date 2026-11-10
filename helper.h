#ifndef _HELPER_H_
#define _HELPER_H_

#undef max

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

#include <cyVector.h>
#include <cyMatrix.h>

using namespace cy;
using std::string;
using std::vector;

typedef unsigned uint;
typedef Vec2<int> Vec2i;
typedef Vec2<uint> Vec2u;
typedef Vec3<int> Vec3i;
typedef Vec3<uint> Vec3u;
typedef Vec4<int> Vec4i;
typedef Vec4<uint> Vec4u;

#define MAX_UINT std::numeric_limits<uint>::max()

// Useful constants
const Vec3f WORLD_Y = Vec3f(0.0f, 1.0f, 0.0f);
const Vec3f WORLD_X = Vec3f(1.0f, 0.0f, 0.0f);
const Vec3f WORLD_Z = Vec3f(0.0f, 0.0f, 1.0f);
const Vec3f ORIGIN = Vec3f(0.0f);

enum Axis : uint8_t { X = 0b001, Y = 0b010, Z = 0b100, X_NEG = 0b1001, Y_NEG = 0b1010, Z_NEG = 0b1100, NEG = 0b1000  };

// Compute distance of a point (pt) from a line (linePt + t * lineDir) 
// using Lagrange's identity
inline float DistToLine(const Vec3f& pt, const Vec3f& linePt, const Vec3f& lineDir) {
	Vec3f relPt = pt - linePt;
	float dot = relPt.Dot(lineDir);
	float dSqrd = relPt.LengthSquared() - ((dot * dot) / lineDir.LengthSquared());
	return std::sqrt(dSqrd);
}

/// <summary> Convert degrees to radians </summary>
/// <param name="degrees"> An angle in degrees </param>
/// <returns> An equivalent angle in radians </returns>
inline float degreesToRadians(float degrees) {
	return degrees * (cy::Pi<float>() / 180.0f);
}

/// <summary> Convert radians to degrees </summary>
/// <param name="radians"> An angle in radians </param>
/// <returns> An equivalent angle in degrees </returns>
inline float radiansToDegrees(float radians) {
	return radians * (180.0f / cy::Pi<float>());
}

/// <summary> Compares two floats and returns whether they're equal </summary>
/// <param name="x"> First float to be compared </param>
/// <param name="y"> Second float to be compared </param>
/// <param name="epsilon"> Float for precision </param>
/// <returns> Boolean whether the distance is equal </returns>
inline bool compareFloat(float x, float y, float epsilon) {
	float dist = std::abs(x - y);
	return dist < epsilon;
}

inline Vec3f sphericalToCartesian(const Vec2f& coords) {
	float azimuthalAngle = degreesToRadians(coords.x);
	float polarAngle = degreesToRadians(coords.y);
	float sinPolar = std::sin(polarAngle);

	Vec3f cartesian;
	cartesian.x = std::cos(azimuthalAngle) * sinPolar;
	cartesian.y = std::cos(polarAngle);
	cartesian.z = -std::sin(azimuthalAngle) * sinPolar;

	return cartesian;
}

/// <summary> Converts spherical coordinates to Cartesian </summary>
/// <param name="coords"> Vector of coordinates to convert </param>
/// <returns> Vector of Cartesian coordinates that have been converted </returns>
inline Vec3f sphericalToCartesian(const Vec3f& coords) {
	return sphericalToCartesian(Vec2f(coords)) * coords.z;
}


inline void clamp(float& x, float left, float right) {
	if (x < left) { x = left; }
	else if (x > right) { x = right; }
}

inline uint WrapDec(uint x, uint wrapTo, uint wrapFrom = 0) {
	if (x == wrapFrom) {
		return wrapTo;
	}
	return x - 1;
}

inline uint WrapSub(uint x, uint sub, uint wrapTo, uint wrapFrom = 0) {
	uint range = wrapTo - wrapFrom + 1;
	uint offset = (wrapTo - x + sub) % range;
	return wrapTo - offset;
}

inline uint WrapInc(uint x, uint wrapFrom, uint wrapTo = 0) {
	if (x == wrapFrom) {
		return wrapTo;
	}
	return x + 1;
}

inline uint WrapAdd(uint x, uint add, uint wrapFrom) {
	return (x + add) % (wrapFrom + 1);
}

inline uint WrapAdd(uint x, uint add, uint wrapFrom, uint wrapTo) {
	uint range = wrapFrom - wrapTo + 1;
	uint offset = (x + add - wrapTo) % range;
	return wrapTo + offset;
}

inline void combineHash(size_t& base, const size_t& newHash) {
	base = (base << 1) + base + newHash;
}

/// <summary> Creates a vector structure with three elements </summary>
/// <typeparam name="T"> Generic type inside the vector </typeparam>
template<typename T>
struct std::hash<cy::Vec3<T>> {
	size_t operator()(const cy::Vec3<T>& vec) const noexcept {
		std::hash<T> hash;
		size_t val = hash(vec.x);
		combineHash(val, hash(vec.y));
		combineHash(val, hash(vec.z));
		return val;
	}
};

/// <summary> Creates a vector structure with two elements </summary>
/// <typeparam name="T"> Generic type inside the vector </typeparam>
template<typename T>
struct std::hash<cy::Vec2<T>> {
	size_t operator()(const cy::Vec2<T>& vec) const noexcept {
		std::hash<T> hash;
		size_t val = hash(vec.x);
		combineHash(val, hash(vec.y));
		return val;
	}
};

inline uint64_t packUints(uint x, uint y) { return (uint64_t(x) << 32) | uint64_t(y); }

template<>
struct std::hash<Vec2u> {
	size_t operator()(const Vec2u& vec) const noexcept { return std::hash<uint64_t>{}(packUints(vec.x, vec.y)); }
};

// Vector Output Stream Overloads
inline std::ostream& operator<<(std::ostream& out, const Vec4f& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec4u& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec4i& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec3f& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec3u& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec3i& vec) {
	out << vec.x << ", " << vec.y << ", " << vec.z;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec2f& vec) {
	out << vec.x << ", " << vec.y;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec2u& vec) {
	out << vec.x << ", " << vec.y;
	return out;
}

inline std::ostream& operator<<(std::ostream& out, const Vec2i& vec) {
	out << vec.x << ", " << vec.y;
	return out;
}

/// <summary> Converts a rotation matrix to Euler angles </summary>
/// <param name="mat"> Rotation Matrix to be converted </param>
/// <returns> Euler angles produced from the rotation matrix </returns>
inline Vec3f matrixToEulerAngle(const Matrix3f& mat) {
	// Refer to https://www.geometrictools.com/Documentation/EulerAngles.pdf
	
	Vec3f eulerRot;
	eulerRot.y = std::asin(mat.cell[6]);

	constexpr float HALF_PI = Pi<float>() / 2.0f;
	constexpr float EPSILON = 0.0001f;

	if (eulerRot.y < HALF_PI - EPSILON && eulerRot.y > -HALF_PI + EPSILON) {
		eulerRot.x = std::atan2(-mat.cell[7], mat.cell[8]);
		eulerRot.z = std::atan2(-mat.cell[3], mat.cell[0]);
	}
	else if (eulerRot.y >= HALF_PI - EPSILON && eulerRot.y <= HALF_PI + EPSILON) {
		eulerRot.x = std::atan2(mat.cell[1], mat.cell[4]);
		eulerRot.z = 0.0f;
	}
	else if (eulerRot.y <= -HALF_PI + EPSILON && eulerRot.y >= -HALF_PI - EPSILON) {
		eulerRot.z = std::atan2(mat.cell[1], mat.cell[4]);
		eulerRot.x = 0.0f;
	}

	eulerRot.x = radiansToDegrees(eulerRot.x);
	eulerRot.y = radiansToDegrees(eulerRot.y);
	eulerRot.z = radiansToDegrees(eulerRot.z);

	return eulerRot;
}

/// <summary> Converts an axis angle to a Euler angle </summary>
/// <param name="axis"> Axis to be converted </param>
/// <param name="angle"> Angle to be converted </param>
/// <returns> Vector of Euler Angle floats </returns>
inline Vec3f axisAngleToEulerAngle(const Vec3f& axis, float angle) {
	Matrix3f mat;
	mat.SetRotation(axis, angle);

	return matrixToEulerAngle(mat);
}

inline Vec3f vectorProj(const Vec3f& v, const Vec3f& u) {
	return (v.Dot(u) / u.LengthSquared()) * u;
}

inline Vec3f unitVectorProj(const Vec3f& v, const Vec3f& u) {
	return v.Dot(u) * u;
}

inline Vec3f projectToPlane(const Vec3f& v, const Vec3f& n) {
	return v - unitVectorProj(v, n);
}

inline Vec2f projectToLocalPlane(const Vec3f& p, const Vec3f& n) {
	uint component = abs(n.y) < 0.9f ? 1 : 0;
	Vec3f uBasis(0.0f);
	uBasis[component] = 1.0f;

	uBasis = n.Cross(uBasis);
	uBasis.Normalize();
	Vec3f vBasis = n.Cross(uBasis);

	return Vec2f(p.Dot(uBasis), p.Dot(vBasis));
}


#endif
