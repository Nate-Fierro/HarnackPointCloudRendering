#ifndef _POINT_CLOUD_PARSER_H_
#define _POINT_CLOUD_PARSER_H_

#include <algorithm>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <unordered_set>
#include <vector>
#include <cyPointCloud.h>
#include <Eigen/SVD>
#include "helper.h"

extern "C" {
#define SINGLE
#define REAL float
#define TRILIBRARY
#define ANSI_DECLARATORS
#include "triangle.h"
}

inline void Free(triangulateio& io) {
    trifree(io.pointlist);
    trifree(io.pointattributelist);
    trifree(io.pointmarkerlist);
    trifree(io.trianglelist);
    trifree(io.triangleattributelist);
    trifree(io.trianglearealist);
    trifree(io.neighborlist);
    trifree(io.segmentlist);
    trifree(io.segmentmarkerlist);
    trifree(io.holelist);
    trifree(io.regionlist);
    trifree(io.edgelist);
    trifree(io.edgemarkerlist);
    trifree(io.normlist);
    io = {};
}

inline float PolygonAreaCCW(const std::vector<std::pair<float, Vec2f>>& polygon) {
    if (polygon.size() < 3) return 0.0f;
    double area = 0.0;
    Vec2f prev = polygon.back().second;
    for (const auto& vertex : polygon) {
        const Vec2f& curr = vertex.second;
        area += double(prev.x) * curr.y - double(prev.y) * curr.x;
        prev = curr;
    }
    return float(area * 0.5);
}

using PtCloud3f = cy::PointCloud<Vec3f, float, 3u>;

class CustomPtCloud : protected PtCloud3f {
    std::vector<Vec3f> pos;
    std::vector<Vec3f> norm;
    std::vector<float> area;
    bool precomputedAreas = false;

    bool LoadAreas(const std::filesystem::path& path) {
        std::ifstream input(path);
        std::string line;
        auto nextLine = [&]() {
            while (std::getline(input, line)) {
                const auto start = line.find_first_not_of(" \t\r");
                if (start != std::string::npos && line[start] != '#') return true;
            }
            return false;
        };
        if (!input || !nextLine() || line != "HARNACK_POINT_AREAS 1") return false;
        size_t count = 0;
        if (!nextLine() || !(std::istringstream(line) >> count) || count != pos.size()) return false;
        area.resize(count);
        for (size_t i = 0; i < count; ++i) {
            Vec3f p;
            if (!nextLine() || !(std::istringstream(line) >> p.x >> p.y >> p.z >> area[i])) return false;
        }
        precomputedAreas = true;
        return true;
    }

    bool ApproxAreas(std::ostream* outStream) {

        constexpr uint K = 20;
        area.clear();
        area.reserve(pos.size());
        for (uint i = 0; i < uint(pos.size()); ++i) {
            PointInfo pts[K];
            const uint count = GetPoints(i, K, pts);
            if (count < 3) {
                if (outStream) *outStream << "ERROR: Point " << i << " has less than three neighbors with good normals\n";
                return false;
            }
            Vec3f avg(0.0f);
            uint target = count;
            float radiusSquared = 0.0f;
            for (uint j = 0; j < count; ++j) {
                avg += pts[j].pos;
                if (pts[j].index == i) target = j;
                radiusSquared = std::max(radiusSquared, pts[j].distanceSquared);
            }
            avg /= float(count);

            // A neighborhood can have fewer than K points
            Eigen::Matrix<float, Eigen::Dynamic, 3> mat(count, 3);
            for (uint j = 0; j < count; ++j)
                for (uint k = 0; k < 3; ++k)
                    mat(j, k) = pts[j].pos[k] - avg[k];
            Eigen::JacobiSVD<decltype(mat)> svd(mat, Eigen::ComputeFullV);
            Vec3f fit(svd.matrixV().col(2).data());
            fit.Normalize();
            Vec2f pts2D[K];
            for (uint j = 0; j < count; ++j)
                pts2D[j] = projectToLocalPlane(pts[j].pos - pos[i], fit);

            triangulateio in{}, out{}, vorout{};
            in.numberofpoints = int(count);
            in.pointlist = &pts2D[0].x;
            char cmd[] = "zevQ";
            triangulate(cmd, &in, &out, &vorout);

            std::vector<std::pair<float, Vec2f>> verts;
            std::unordered_set<int> vertSet;
            bool unbounded = false;
            for (int j = 0; j < vorout.numberofedges; ++j) {
                const int e = j * 2;
                if (out.edgelist[e] != int(target) && out.edgelist[e + 1] != int(target)) continue;
                for (int k = 0; k < 2; ++k) {

                    const int v = vorout.edgelist[e + k];
                    if (v < 0) { unbounded = true; continue; }
                    if (!vertSet.insert(v).second) continue;
                    const Vec2f p = Vec2f(vorout.pointlist + 2 * v) - pts2D[target];
                    verts.emplace_back(std::atan2(p.y, p.x), p);
                }
            }
            std::sort(verts.begin(), verts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
            float a = std::abs(PolygonAreaCCW(verts));
            // open-boundary cells are infinite. Fallback to a finite local-density estimate
            if (unbounded || a <= 0.0f)
                a = cy::Pi<float>() * radiusSquared / float(count - 1);
            Free(out);
            Free(vorout);
            area.push_back(a);
        }
        return true;
    }

public:
    int GetPoints(uint pt, float radius, uint maxCount, PointInfo* closestPoints) const {

        const Vec3f position = pos[pt];
        const Vec3f normal = norm[pt];
        uint pointsFound = 0;
        PtCloud3f::GetPoints(position, radius, [&](uint i, Vec3f const& p, float d2, float& r2) {

            if (norm[i].Dot(normal) <= 0.0f) return;
            if (pointsFound == maxCount) {

                std::pop_heap(closestPoints, closestPoints + maxCount);
                closestPoints[maxCount - 1].index = i;
                closestPoints[maxCount - 1].pos = p;
                closestPoints[maxCount - 1].distanceSquared = d2;
                std::push_heap(closestPoints, closestPoints + maxCount);
                r2 = closestPoints[0].distanceSquared;
            } else {

                closestPoints[pointsFound].index = i;
                closestPoints[pointsFound].pos = p;
                closestPoints[pointsFound++].distanceSquared = d2;
                if (pointsFound == maxCount) {
                    std::make_heap(closestPoints, closestPoints + maxCount);
                    r2 = closestPoints[0].distanceSquared;
                }
            }
        });
        return int(pointsFound);
    }

    int GetPoints(uint pt, uint maxCount, PointInfo* closestPoints) const {

        return GetPoints(pt, std::numeric_limits<float>::max(), maxCount, closestPoints);
    }

    // obj: one vn per v
    bool LoadFromFileObj(const char* filename, std::ostream* outStream = &std::cout, bool usePrecomputedAreas = true) {
        pos.clear(); norm.clear(); area.clear();
        precomputedAreas = false;
        std::ifstream input(filename);
        auto fail = [&](const std::string& message) {
            if (outStream) *outStream << "ERROR: " << message << '\n';
            return false;
        };
        if (!input) return fail(std::string("cannot open file ") + filename);
        std::string line;
        while (std::getline(input, line)) {
            std::istringstream row(line);
            std::string tag;
            row >> tag;
            if (tag != "v" && tag != "vn") continue;
            Vec3f v;
            if (!(row >> v.x >> v.y >> v.z)) return fail("i8nvalid point record");
            if (tag == "v") pos.push_back(v);
            else norm.push_back(v / v.Length());
        }

        if (pos.empty() || norm.size() != pos.size()) return fail("Expected one normal per point");
        PtCloud3f::Build(uint32_t(pos.size()), pos.data());
        auto areaPath = std::filesystem::path(filename);
        areaPath.replace_extension(".areas");
        if (usePrecomputedAreas && std::filesystem::exists(areaPath)) {
            if (!LoadAreas(areaPath)) return fail("Cannot read point areas");
        } else if (!ApproxAreas(outStream)) return fail("Cannot estimate point areas");
        return true;
    }

    void Normalize() {
        Vec3f lo = pos.front(), hi = pos.front();
        for (const auto& p : pos)
            for (uint k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
        const Vec3f center = (lo + hi) * 0.5f;
        float radius = 0.0f;
        for (const auto& p : pos) radius = std::max(radius, (p - center).Length());
        for (auto& p : pos) p = (p - center) / radius;
        for (auto& a : area) a /= radius * radius;
        PtCloud3f::Build(uint32_t(pos.size()), pos.data());
    }

    const std::vector<Vec3f>& Positions() const { return pos; }
    const std::vector<Vec3f>& Normals() const { return norm; }
    const std::vector<float>& Areas() const { return area; }
    bool HasPrecomputedAreas() const { return precomputedAreas; }
};

#endif
