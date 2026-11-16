#include "surface_areas.h"


// this is for precomputing the areas totally separate form harnack tracing
Job ReadJob(const char* filename) {
    std::ifstream in(filename);
    std::string tag;
    int version;
    size_t nv,nf,ns;
    if (!(in>>tag>>version>>nv>>nf>>ns) || tag!="HARNACK_AREA_JOB" || version!=1)
        throw std::runtime_error("Invalid mesh area job");
    Job job;
    job.vertices.resize(nv); job.faces.resize(nf); job.seeds.resize(ns);
    for (auto& p:job.vertices) in>>p.x>>p.y>>p.z;
    for (auto& f:job.faces) {
        in>>f[0]>>f[1]>>f[2];
    }
    for (auto& s:job.seeds) {
        in>>s.face>>s.position.x>>s.position.y>>s.position.z>>s.identity.x>>s.identity.y>>s.identity.z;
    }
    if(!in) throw std::runtime_error("Truncated mesh area job");
    return job;
}

int main(int argc,char** argv) {
    try {
        if(argc<3) throw std::runtime_error("Usage: precompute_areas mesh-job.txt output.areas [depth=8]");
        int depth=argc>3 ? std::stoi(argv[3]) : 8;
        if(depth<3 || depth>12) throw std::runtime_error("Depth must be in [3,12]");
        Job job=ReadJob(argv[1]);
        const auto start=std::chrono::steady_clock::now();
        SurfaceAreas solver(job);
        auto coarse=solver.Compute(depth-2), fine=solver.Compute(depth);
        double difference=0,maxRelative=0;
        for(size_t i=0;i<fine.areas.size();++i) {
            double delta=std::abs(coarse.areas[i]-fine.areas[i]);
            difference+=delta;
            maxRelative=std::max(maxRelative,delta/fine.areas[i]);
        }
        double l1=difference/fine.totalArea;
        if(l1>.001 || maxRelative>.02) throw std::runtime_error("Area integration has not converged; increase depth");
        std::ofstream out(argv[2]);
        out<<std::setprecision(17)
           <<"# Mesh-geodesic Voronoi areas: Kirsanov MMP distances, adaptive surface integration\n"
           <<"# Fine/coarse depths: "<<depth<<' '<<depth-2<<"\n"
           <<"# Source mesh area: "<<fine.totalArea<<"\n"
           <<"# Relative L1 area change: "<<l1<<"\n"
           <<"# Maximum per-point relative area change: "<<maxRelative<<"\n"
           <<"# Mixed leaf area fraction: "<<fine.unresolvedArea/fine.totalArea<<"\n"
           <<"HARNACK_POINT_AREAS 1\n"<<fine.areas.size()<<'\n';
        for(size_t i=0;i<fine.areas.size();++i) {
            V p=job.seeds[i].identity;
            out<<p.x<<' '<<p.y<<' '<<p.z<<' '<<fine.areas[i]<<'\n';
        }
        if(!out) throw std::runtime_error("Cannot write area output");
        std::cout<<std::setprecision(10)<<"Saved "<<fine.areas.size()<<" areas; total="<<fine.totalArea
                 <<"; relative L1 change="<<l1<<"; max relative change="<<maxRelative
                 <<"; mixed leaf fraction="<<fine.unresolvedArea/fine.totalArea
                 <<"; queries="<<fine.queries<<"; seconds="
                 <<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
        return 0;
    } catch(const std::exception& e) { std::cerr<<"ERROR: "<<e.what()<<'\n'; return 1; }
}
