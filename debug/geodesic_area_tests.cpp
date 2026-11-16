#include "surface_areas.h"

void SelfTest() {
    Job job;
    job.vertices={{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}};
    job.faces={{{0,1,2}},{{0,2,3}}};
    job.seeds={{1,{-.53,0,0},{-.53,0,0}},{0,{.25,0,0},{.25,0,0}}};
    SurfaceAreas solver(job);
    auto result=solver.Compute(12);
    // The planar Voronoi bisector is x=-0.14, giving areas 1.72 and 2.28.
    std::cout << "Planar areas: " << std::setprecision(12) << result.areas[0] << ", " << result.areas[1] << '\n';
    if(std::abs(result.areas[0]-1.72)>.0005 || std::abs(result.areas[1]-2.28)>.0005)
        throw std::runtime_error("Analytic planar Voronoi area regression failed");
    std::cout<<"Geodesic area regression passed: "<<result.areas[0]<<", "<<result.areas[1]<<'\n';
}

int main() {
    try { SelfTest(); return 0; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
