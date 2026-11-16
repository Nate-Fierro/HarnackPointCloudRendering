#version 450 core

// TODO: build/upload the hierarchy and point-index buffer on the CPU.
// TODO: add higher-order moments and an error-based cluster acceptance rule.
// TODO: bounds for Harnack stepping of the approximate field

layout(location = 0) out vec4 color;
struct CloudPoint { vec3 p; float a; vec3 n; };
struct Camera { vec3 pos; vec3 f; vec3 u; vec3 r; };
struct Ray { vec3 p; vec3 d; };
layout(std430, binding = 0) readonly buffer PointCloudBuffer { CloudPoint ptCld[]; };
uniform uvec2 res;
uniform float fov;
uniform Camera cam;

struct Cluster {
    vec3 center; float radius;       // Area-weighted center / enclosing radius.
    vec3 areaNormal; uint next;      // SUM(a_i * n_i). this is not a unit normal
    uint firstPoint; uint pointCount; uvec2 padding;
};
layout(std430, binding = 1) readonly buffer ClusterBuffer { Cluster clusters[]; };
layout(std430, binding = 2) readonly buffer PointIndexBuffer { uint pointIndices[]; };

const float ISO_VALUE = 0.5;
const float FIELD_EPSILON = 0.0001;
// larger ratio opens more clusters
const float OPENING_RATIO = 1.5;
const float PREVIEW_STEP = 0.025;

float DipoleContribution(vec3 areaNormal, vec3 d) {
    float distSquared = max(dot(d, d), 1e-12);
    return dot(areaNormal, d) * inversesqrt(distSquared) / distSquared;
}

float FastDipolePotential(vec3 x) {
    float winding = 0.0;
    uint node = 0u;
    while (node < uint(clusters.length())) {
        Cluster cluster = clusters[node];
        vec3 d = cluster.center - x;
        float openingDistance = OPENING_RATIO * cluster.radius;
        if (dot(d, d) > openingDistance * openingDistance) {
            // Replace the whole subtree by one dipole
            winding += DipoleContribution(cluster.areaNormal, d);
            node = cluster.next;
        } else if (cluster.pointCount > 0u) {
            // Nearby leaves use the original point contributions
            for (uint i = 0u; i < cluster.pointCount; ++i) {
                CloudPoint pt = ptCld[pointIndices[cluster.firstPoint + i]];
                winding += DipoleContribution(pt.a * pt.n, pt.p - x);
            }
            node = cluster.next;
        } else {
            ++node; 
        }
    }
    return winding;
}

float SignedField(vec3 x) {
    return ISO_VALUE - FastDipolePotential(x);
}

Ray GenerateRayThroughPixel() {
    float halfHeight = tan(fov * 0.5);
    vec2 uv = 2.0 * gl_FragCoord.xy / vec2(res) - 1.0;
    vec3 d = cam.f + halfHeight * (uv.x * float(res.x) / float(res.y) * cam.r + uv.y * cam.u);
    return Ray(cam.pos, normalize(d));
}

// this doesn't work yet especially since harnack doesnt consider the approximation error
float TraceRay(Ray ray) {
    float t = 0.0;
    float value = SignedField(ray.p);
    for (int iter = 0; iter < 2000; ++iter) {
        if (abs(value) < FIELD_EPSILON) return t;
        float nextT = t + PREVIEW_STEP;
        float nextValue = SignedField(ray.p + nextT * ray.d);
        if ((value < 0.0) != (nextValue < 0.0)) {
            float lo = t, hi = nextT;
            float flo = value;
            for (int i = 0; i < 8; ++i) {
                float mid = 0.5 * (lo + hi);
                float fm = SignedField(ray.p + mid * ray.d);
                if ((fm < 0.0) == (flo < 0.0)) { lo = mid; flo = fm; }
                else hi = mid;
            }
            return 0.5 * (lo + hi);
        }
        t = nextT;
        value = nextValue;
    }
    return -1.0;
}

vec3 SurfaceNormal(vec3 x) {

    const float h = 0.001;
    vec3 gradient = vec3(
        SignedField(x + vec3(h, 0.0, 0.0)) - SignedField(x - vec3(h, 0.0, 0.0)),
        SignedField(x + vec3(0.0, h, 0.0)) - SignedField(x - vec3(0.0, h, 0.0)),
        SignedField(x + vec3(0.0, 0.0, h)) - SignedField(x - vec3(0.0, 0.0, h))
    );
    return dot(gradient, gradient) > 1e-20 ? normalize(gradient) : vec3(0.0, 1.0, 0.0);
}

void main() {
    Ray ray = GenerateRayThroughPixel();
    float t = TraceRay(ray);
    if (t < 0.0) { color = vec4(0.035, 0.055, 0.085, 1.0); return; }
    vec3 n = SurfaceNormal(ray.p + t * ray.d);
    float diffuse = max(dot(n, normalize(vec3(-0.5, 0.8, 1.0))), 0.0);
    float lighting = 0.28 + 0.72 * diffuse;
    color = vec4(vec3(0.22, 0.72, 0.9) * lighting, 1.0);
}
