#version 450 core

layout(location = 0) out vec4 color;

struct CloudPoint {
	vec3 p;
	float a;
	vec3 n;
};

struct Camera {
	vec3 pos;
	vec3 f;
	vec3 u;
	vec3 r;
};

struct Ray {
	vec3 p;
	vec3 d;
};

struct Transform {
    float scale;
    vec3 pos;
};

layout(std430, binding = 0) readonly buffer PointCloudBuffer
{
    CloudPoint ptCld[];
};

uniform uvec2 res;
uniform float fov;
uniform Camera cam;
//uniform Transform ptCldTrans;

const float tMax = 50.0f;
const float INFINITY = 1e30; 

float MinimumDistance(in vec3 x, out uint clstPt){
	float minDistSqrd = 1e30f;
	clstPt = 0;

	for(uint i = 0; i < ptCld.length(); ++i){
		vec3 d = ptCld[i].p - x;
		float distSqrd = dot(d, d);

		bool lt = distSqrd < minDistSqrd;

		minDistSqrd	= float(lt) * distSqrd	+ float(!lt)	* minDistSqrd;
		clstPt		= uint(lt)	* i			+ uint(!lt)		* clstPt;
	}

	return sqrt(minDistSqrd);
}

/*float HarmonicBoundFunc(in vec3 x, in float r){
	float c = 0.0f;

	for(uint i = 0; i < ptCld.length(); ++i){
		CloudPoint pt = ptCld[i];
		
		vec3 d = pt.p - x;

		float num	= (dot(d, pt.n) - r);
		float denom	= length(d) + sign(num) * r;

		c += pt.a * (num / denom);
	}

	return c;
}*/

float DipolePotential(in vec3 x, out float r, out float c, out uint clstPt, out float minDist){
	minDist = MinimumDistance(x, clstPt);
	r = 0.25f * minDist;

	c = 0.0f;
	float f = 0.0f;

	if(minDist < 0.0001f){
		return INFINITY;
	}

	for(uint i = 0; i < ptCld.length(); ++i){
		CloudPoint pt = ptCld[i];
		
		vec3 d		= pt.p - x;
		float dDotN	= dot(d, pt.n);
		float dLen	= length(d);

		f += (pt.a * dDotN) / (dLen * dLen * dLen);

		float c_num		= (dDotN - r);
		float c_denom	= dLen + sign(c_num) * r;
		c_denom *= c_denom * c_denom;

		c += pt.a * (c_num / c_denom);
	}

	return f;
}

Ray GenerateRayThroughPixel(){
	float imgPlaneH = tan(fov / 2.0f);				// half height
	float imgPlaneW = imgPlaneH * (float(res.x) / float(res.y));	// half width

	vec3 pxOffset = cam.pos - (imgPlaneW * cam.r) - (imgPlaneH * cam.u) + cam.f;
	float s = (imgPlaneW * 2.0f) / res.x;

	vec3 px = pxOffset + (s * gl_FragCoord.x) * cam.r + (s * gl_FragCoord.y) * cam.u;

	Ray ray;
	ray.p = cam.pos;
	ray.d = normalize(px - cam.pos);

	return ray;
}

float TraceRay(in Ray ray){
	float t = 0.0f;
	uint iter = 0;

	do {

		vec3 curr = ray.p + (t * ray.d);

		float c, r, minDist;
		uint clstPt;
		float f = DipolePotential(curr, r, c, clstPt, minDist);

		if (f < 0.0001f || minDist < 0.0001f){
			return t;
		}

		float step = 0.0f;

		if (c >= 0.0f) {
			step = r;
		}
		else {
			float a = (f - c) / -c;
			step = 0.5f * r * abs(a + 2.0f - sqrt(a * a + 8 * a));
		}

		if(step < 0.0001f || isnan(step) || isinf(step)){
			return -1.0f;
		}

		t += step;
		iter++;

	} while(t < tMax && iter < 1000);

	return -1.0f;
}

void main(){
	Ray ray = GenerateRayThroughPixel();
	float t = TraceRay(ray);

	if(t < 0.0f){
		color = vec4(0.0f, 1.0f, 0.0f, 1.0f);
		return;
	}

	color = vec4(1.0f, 0.0f, 0.0f, 1.0f);
}

