#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "helper.h"

enum CameraType { ORBIT, FREE };

class AbstractCamera {
public:
	virtual Vec3f position() const = 0;
	virtual void setPosition(const Vec3f& pos) = 0;
	virtual Matrix4f transform() const = 0;
	virtual void transform(float data[16]) const = 0;

	virtual Vec3f forward() const = 0;
	virtual void directions(Vec3f& f, Vec3f& u, Vec3f& r) const {
		f = forward();

		r = WORLD_Y.Cross(f);
		r.Normalize();

		u = r.Cross(f);
		u.Normalize();
	}

	virtual Vec3f right() const { Vec3f r = WORLD_Y.Cross(forward()); r.Normalize(); return r; }
	virtual Vec3f up() const { Vec3f f, u, r; directions(f, u, r); return u; }

	virtual void mouseDrag(const Vec2f& drag) = 0;
	virtual void mouseWheel(float dx, float dy) = 0;
};

class OrbitCamera : public AbstractCamera {
	Vec3f origin = Vec3f(0.0f); // World-Space Center of Orbit
	Vec3f spherical = Vec3f(0.0f, 90.0f, 1.0f); // Local Spherical Coordinates
	Vec3f cartesian = Vec3f(1.0f, 0.0f, 0.0f); // Local Cartesian Coordinates (cached)

public:

	OrbitCamera() { }

	void setOrigin(const Vec3f& origin) { this->origin = origin; }
	Vec3f getOrigin() const { return origin; }
	void setSphericalPosition(const Vec3f& pos) {
		spherical = pos;
		cartesian = sphericalToCartesian(pos);
	}

	Vec3f sphericalPosition() { return spherical; }
	Vec3f position() const override { return cartesian + origin; }

	void setPosition(const Vec3f& pos) override {
		cartesian = pos - origin;

		float radius = cartesian.Length();

		spherical.x = radiansToDegrees(std::acos(pos.x / radius));
		spherical.y = radiansToDegrees(std::acos(pos.y / radius));
		spherical.z = radius;
	}

	Matrix4f transform() const override { Matrix4f m; m.SetView(position(), origin, WORLD_Y); return m; }
	void transform(float data[16]) const override { Matrix4f m; m.SetView(position(), origin, WORLD_Y); m.Get(data); }

	Vec3f forward() const override { return (-cartesian / spherical.z); }

	void mouseDrag(const Vec2f& drag) override {
		spherical.x -= drag.x;
		spherical.y += drag.y;
		clamp(spherical.y, 1.0f, 179.0f);

		cartesian = sphericalToCartesian(spherical);
	}

	void mouseWheel(float dx, float dy) override {
		spherical.z -= dy;
		if (spherical.z < 0.00001f) { spherical.z = 0.00001f; }

		spherical.x += dx;

		cartesian = sphericalToCartesian(spherical);
	}
};

class FreeCamera : public AbstractCamera {
	Vec3f pos = Vec3f(0.0f);
	Vec2f dir = Vec2f(0.0f, 90.0f);

public:

	FreeCamera() {}

	void setPosition(const Vec3f& pos) override { this->pos = pos; }
	void setForward(const Vec3f& f) {
		assert(f.LengthSquared() > 1.0f - 0.000001f && f.LengthSquared() < 1.0f + 0.000001f);

		dir.x = radiansToDegrees(std::atan2(-f.z, f.x));
		dir.y = radiansToDegrees(std::acos(f.y));
	}

	Vec3f position() const override { return pos; }
	Vec3f target() const { return pos + sphericalToCartesian(dir); }

	Matrix4f transform() const override { Matrix4f m; m.SetView(pos, target(), WORLD_Y); return m; }
	void transform(float data[16]) const override { Matrix4f m; m.SetView(pos, target(), WORLD_Y); m.Get(data); }

	Vec3f forward() const override { return sphericalToCartesian(dir); }

	void mouseDrag(const Vec2f& drag) override {
		dir.x -= drag.x;
		dir.y -= drag.y;
		clamp(dir.y, 1.0f, 179.0f);
	}

	void mouseWheel(float dx, float dy) override {
		Vec3f f = forward();

		Vec3f r = WORLD_Y.Cross(f);
		r.Normalize();

		pos += r * dx;
		pos += f * -dy;
	}
};

#endif
