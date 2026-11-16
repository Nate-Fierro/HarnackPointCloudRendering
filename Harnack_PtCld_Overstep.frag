#version 450 core

layout(location = 0) out vec4 color;
struct CloudPoint { vec3 p; float a; vec3 n; };
struct Camera { vec3 pos; vec3 f; vec3 u; vec3 r; };
struct Ray { vec3 p; vec3 d; };
layout(std430, binding = 0) readonly buffer PointCloudBuffer { CloudPoint ptCld[]; };
uniform uvec2 res;
uniform float fov;
uniform Camera cam;

const float INV_FOUR_PI = 0.07957747154594767;
const float ISO_VALUE = 0.5;
const float FIELD_EPSILON = 0.0001;
const float SOURCE_EPSILON = 0.00001;

// f = |iso - winding|. c is a lower bound on the same signed branch in
// the ball B(x,r).
float DipolePotential(vec3 x, out float r, out float c, out float minDist) {
    float minDistSquared = 1e30;
    for (int i = 0; i < ptCld.length(); ++i) {
        vec3 d = ptCld[i].p - x;
        minDistSquared = min(minDistSquared, dot(d, d));
    }
    minDist = sqrt(minDistSquared);
    r = 0.25 * minDist;
    c = 0.0;
    if (minDist < SOURCE_EPSILON) return 0.0;

    float winding = 0.0, lower = 0.0, upper = 0.0;
    for (int i = 0; i < ptCld.length(); ++i) {
        CloudPoint pt = ptCld[i];
        vec3 d = pt.p - x;
        float dist = length(d);
        float projection = dot(d, pt.n);
        winding += pt.a * projection / (dist * dist * dist);
        float lowerNumerator = projection - r;
        float lowerDenominator = dist + sign(lowerNumerator) * r;
        float upperNumerator = projection + r;
        float upperDenominator = dist - sign(upperNumerator) * r;
        lower += pt.a * lowerNumerator / (lowerDenominator * lowerDenominator * lowerDenominator);
        upper += pt.a * upperNumerator / (upperDenominator * upperDenominator * upperDenominator);
    }
    float value = ISO_VALUE - INV_FOUR_PI * winding;
    c = value >= 0.0 ? ISO_VALUE - INV_FOUR_PI * upper : INV_FOUR_PI * lower - ISO_VALUE;
    c -= 0.000001;
    return abs(value);
}

Ray GenerateRayThroughPixel() {
    float halfHeight = tan(fov * 0.5);
    vec2 uv = 2.0 * gl_FragCoord.xy / vec2(res) - 1.0;
    vec3 d = cam.f + halfHeight * (uv.x * float(res.x) / float(res.y) * cam.r + uv.y * cam.u);
    return Ray(cam.pos, normalize(d));
}

float SignedField(vec3 x) {
    float winding = 0.0;
    for (int i = 0; i < ptCld.length(); ++i) {
        vec3 d = ptCld[i].p - x;
        float dist = length(d);
        winding += ptCld[i].a * dot(d, ptCld[i].n) / max(dist * dist * dist, 1e-30);
    }
    return ISO_VALUE - INV_FOUR_PI * winding;
}

float RefineStalledRay(Ray ray, float t) {
    // near a point source the field is very steep. Float ray coordinates can
    // stop changing before the field residual reaches FIELD_EPSILON. Accept
    // only a root bracketed within a small spatial tolerance then refine it
    float tolerance = max(1e-5, 2e-6 * t);
    float lo = max(0.0, t - tolerance), hi = t + tolerance;
    float flo = SignedField(ray.p + lo * ray.d);
    float fhi = SignedField(ray.p + hi * ray.d);
    if ((flo < 0.0) == (fhi < 0.0)) return -1.0;
    for (int i = 0; i < 12; ++i) {
        float mid = 0.5 * (lo + hi);
        if (mid == lo || mid == hi) break;
        float fm = SignedField(ray.p + mid * ray.d);
        if ((fm < 0.0) == (flo < 0.0)) { lo = mid; flo = fm; }
        else hi = mid;
    }
    return 0.5 * (lo + hi);
}

float TraceRay(Ray ray) {
    float t = 0.0;

    float safeStep = 0.0;
    float tSafe = 0.0;
    float overStep = 0.0;

    for (int iter = 0; iter < 1000 && t < 50.0; ++iter) {
        float c, r, minDist;
        float f = DipolePotential(ray.p + t * ray.d, r, c, minDist);
        if (f < FIELD_EPSILON || minDist < SOURCE_EPSILON) return t;

        float step = r;
        if (c < 0.0) {
            // Rationalized Harnack root avoids bad cancellation near hit
            float b = clamp(-c / (f - c), 0.0, 1.0);
            step *= 2.0 * (1.0 - b) / (1.0 + 2.0 * b + sqrt(1.0 + 8.0 * b));
        }

        if(step + safeStep < overStep){
            // revert back
            t = tSafe;
            safeStep = 0.0;
            overStep = -1.0;
            continue;
        }

        float next = t + 0.99 * step;

        if (next <= t) return RefineStalledRay(ray, t);

        safeStep = step;
        tSafe = next;
        overStep = 1.75 * step;

        t += 0.99 * overStep;
    }
    return t < 50.0 ? RefineStalledRay(ray, t) : -1.0;
}

vec3 SurfaceNormal(vec3 x) {
    vec3 gradient = vec3(0.0);
    for (int i = 0; i < ptCld.length(); ++i) {
        CloudPoint pt = ptCld[i];
        vec3 d = pt.p - x;
        float distSquared = max(dot(d, d), 1e-12);
        float inverseCube = inversesqrt(distSquared) / distSquared;
        // Outward normal is -grad(winding).
        gradient += pt.a * inverseCube * (pt.n - 3.0 * dot(d, pt.n) * d / distSquared);
    }
    return dot(gradient, gradient) > 1e-20 ? normalize(gradient) : vec3(0.0, 1.0, 0.0);
}

void main() {
    Ray ray = GenerateRayThroughPixel();
    float t = TraceRay(ray);
    if (t < 0.0) { color = vec4(0.035, 0.055, 0.085, 1.0); return; }
    vec3 pt = ray.p + t * ray.d;
    vec3 n = SurfaceNormal(pt);
    float diffuse = max(dot(n, normalize(cam.pos - pt)), 0.0);
    float lighting = 0.28 + 0.72 * diffuse;
    color = vec4(vec3(0.22, 0.72, 0.9) * lighting, 1.0);
}
