#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <geodesic/geodesic_algorithm_exact.h>

struct V {
    double x = 0, y = 0, z = 0;
    V operator+(V b) const { return {x+b.x, y+b.y, z+b.z}; }
    V operator-(V b) const { return {x-b.x, y-b.y, z-b.z}; }
    V operator*(double b) const { return {x*b, y*b, z*b}; }
};
inline double Dot(V a, V b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline V Cross(V a, V b) { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
struct Seed { unsigned face; V position, identity; };
struct Job {
    std::vector<V> vertices;
    std::vector<std::array<unsigned, 3>> faces;
    std::vector<Seed> seeds;
};
struct Integral {
    std::vector<double> areas;
    double unresolvedArea = 0, totalArea = 0;
    size_t queries = 0;
};

class SurfaceAreas {
    Job job;
    geodesic::Mesh mesh;
    std::unique_ptr<geodesic::GeodesicAlgorithmExact> algorithm;
    std::vector<unsigned> vertexLabels;
    std::vector<std::vector<unsigned>> faceSeeds;
    double scale = 0;
    struct Sample { V p; unsigned label; };

    Sample Query(unsigned face, V p, Integral& result) {
        geodesic::SurfacePoint query(&mesh.faces()[face]);
        query.set(p.x, p.y, p.z);
        double distance;
        const unsigned source = algorithm->best_source(query, distance);
        if (distance >= geodesic::GEODESIC_INF / 2 || source >= job.seeds.size())
            throw std::runtime_error("Mesh component has no reachable point-cloud sample");
        ++result.queries;
        return {p, source};
    }

    bool Contains(V a, V b, V c, V p) const {
        V u=b-a, v=c-a, w=p-a;
        double uu=Dot(u,u), uv=Dot(u,v), vv=Dot(v,v), uw=Dot(u,w), vw=Dot(v,w);
        double det=uu*vv-uv*uv;
        double s=(vv*uw-uv*vw)/det, t=(uu*vw-uv*uw)/det;
        return s >= -1e-9 && t >= -1e-9 && s+t <= 1+1e-9;
    }

    void Integrate(unsigned face, Sample a, Sample b, Sample c, double area,
                   int depth, int maxDepth, Integral& result) {
        Sample center=Query(face, (a.p+b.p+c.p)*(1./3.), result);
        bool same=a.label==b.label && a.label==c.label && a.label==center.label;
        // A tiny cell can be wholly inside a triangle. Explicitly retain every seed.
        if (same)
            for (unsigned seed : faceSeeds[face])
                if (seed != center.label && Contains(a.p,b.p,c.p,job.seeds[seed].position)) { same=false; break; }
        if (same && depth >= 1) { result.areas[center.label] += area; return; }
        if (depth == maxDepth) {
            result.areas[center.label] += area;
            result.unresolvedArea += area;
            return;
        }
        Sample ab=Query(face,(a.p+b.p)*.5,result), bc=Query(face,(b.p+c.p)*.5,result), ca=Query(face,(c.p+a.p)*.5,result);
        Integrate(face,a,ab,ca,area*.25,depth+1,maxDepth,result);
        Integrate(face,ab,b,bc,area*.25,depth+1,maxDepth,result);
        Integrate(face,ca,bc,c,area*.25,depth+1,maxDepth,result);
        Integrate(face,ab,bc,ca,area*.25,depth+1,maxDepth,result);
    }

public:
    explicit SurfaceAreas(Job input) : job(std::move(input)) {
        if (job.vertices.empty() || job.faces.empty() || job.seeds.empty()) throw std::runtime_error("Empty area job");
        V center=job.vertices.front(), lo=center, hi=center;
        for (V p:job.vertices) {
            lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
            hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
        }
        center=(lo+hi)*.5;
        for (V p:job.vertices) scale=std::max(scale,std::sqrt(Dot(p-center,p-center)));
        if (scale<=0) throw std::runtime_error("Degenerate mesh");
        std::vector<double> coordinates;
        std::vector<unsigned> indices;
        for (auto& p:job.vertices) {
            p=(p-center)*(1./scale);
            coordinates.insert(coordinates.end(),{p.x,p.y,p.z});
        }
        for (auto f:job.faces) indices.insert(indices.end(),f.begin(),f.end());
        mesh.initialize_mesh_data(coordinates,indices);
        algorithm = std::make_unique<geodesic::GeodesicAlgorithmExact>(&mesh);
        faceSeeds.resize(job.faces.size());
        std::vector<geodesic::SurfacePoint> sources;
        for (unsigned i=0;i<job.seeds.size();++i) {
            auto& seed=job.seeds[i];
            seed.position=(seed.position-center)*(1./scale);
            geodesic::SurfacePoint point(&mesh.faces()[seed.face]);
            point.set(seed.position.x,seed.position.y,seed.position.z);
            sources.push_back(point);
            faceSeeds[seed.face].push_back(i);
        }
        algorithm->propagate(sources);
        for (auto& v:mesh.vertices()) {
            double distance;
            unsigned label=algorithm->best_source(geodesic::SurfacePoint(&v),distance);
            if (distance>=geodesic::GEODESIC_INF/2 || label>=sources.size())
                throw std::runtime_error("Unseeded disconnected component");
            vertexLabels.push_back(label);
        }
    }

    Integral Compute(int depth) {
        Integral result;
        result.areas.resize(job.seeds.size(),0.);
        for (unsigned f=0;f<job.faces.size();++f) {
            auto ids=job.faces[f];
            V a=job.vertices[ids[0]], b=job.vertices[ids[1]], c=job.vertices[ids[2]];
            V normal=Cross(b-a,c-a);
            double area=.5*std::sqrt(Dot(normal,normal))*scale*scale;
            result.totalArea += area;
            Integrate(f,{a,vertexLabels[ids[0]]},{b,vertexLabels[ids[1]]},{c,vertexLabels[ids[2]]},area,0,depth,result);
        }
        for (double a:result.areas)
            if (!std::isfinite(a) || a<=0) throw std::runtime_error("A sample has zero/nonfinite surface area; increase integration depth");
        double sum=std::accumulate(result.areas.begin(),result.areas.end(),0.);
        if (std::abs(sum-result.totalArea)>1e-9*result.totalArea) throw std::runtime_error("Area conservation failed");
        return result;
    }
};

