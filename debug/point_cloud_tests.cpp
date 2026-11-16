#include "point_cloud.h"
#include "camera.h"
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>

void Check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

double DipolePotential(const CustomPtCloud& cloud, const Vec3f& x) {
    double sum = 0.;
    for (size_t i = 0; i < cloud.Positions().size(); ++i) {
        const Vec3f d = cloud.Positions()[i] - x;
        const double length = d.Length();
        sum += cloud.Areas()[i] * d.Dot(cloud.Normals()[i]) / (length * length * length);
    }
    return sum;
}

int main(int argc, char** argv) {
    try {
        // The omitted closing edge changes this square's area from 4 to 3.
        const std::vector<std::pair<float, Vec2f>> square = {
            {0.f, Vec2f(-1, -1)}, {1.f, Vec2f(1, -1)}, {2.f, Vec2f(1, 1)}, {3.f, Vec2f(-1, 1)}};
        Check(std::abs(PolygonAreaCCW(square) - 4.f) < 1e-6f, "Polygon must include its closing edge");

        CustomPtCloud sphere;
        const auto spherePath = std::filesystem::path(argv[1]) / "sphere-points.obj";
        Check(sphere.LoadFromFileObj(spherePath.string().c_str()), "Sphere must load");
        const double totalArea = std::accumulate(sphere.Areas().begin(), sphere.Areas().end(), 0.);
        Check(std::abs(totalArea - 4 * cy::Pi<double>()) < 0.5, "Unit sphere quadrature area should approximate 4 pi");
        Check(std::abs(DipolePotential(sphere, Vec3f(0.f)) - 4 * cy::Pi<double>()) < 0.5,
              "Inside sphere dipole potential should be 4 pi");
        Check(std::abs(DipolePotential(sphere, Vec3f(3.f, 0.f, 0.f))) < 0.125,
              "Outside sphere dipole potential should be 0");
        for (float a : sphere.Areas()) Check(std::isfinite(a) && a > 0.f, "Point areas must be positive and finite");

        const auto fixture = std::filesystem::current_path() / "point-cloud-test-input.obj";
        { std::ofstream out(fixture); out << "# no final newline\nv -2 -2 0\nv -1 -2 0\nv -2 -1 0\nvn 0 0 2\nvn 0 0 2\nvn 0 0 2"; }
        Check(sphere.LoadFromFileObj(fixture.string().c_str()), "Small planar clouds must load");
        auto areaFixture = fixture;
        areaFixture.replace_extension(".areas");
        { std::ofstream out(areaFixture); out << "HARNACK_POINT_AREAS 1\n3\n-2 -2 0 .25\n-1 -2 0 .5\n-2 -1 0 .75\n"; }
        Check(sphere.LoadFromFileObj(fixture.string().c_str()) && sphere.HasPrecomputedAreas(), "Stored areas must load");
        sphere.Normalize();
        Check(std::abs(std::accumulate(sphere.Areas().begin(),sphere.Areas().end(),0.)-3.)<1e-5,
              "Normalization must scale stored areas by inverse squared radius");
        std::filesystem::remove(areaFixture);
        std::filesystem::remove(fixture);

        OrbitCamera camera;
        camera.setOrigin(Vec3f(1.f, 2.f, 3.f));
        camera.setPosition(Vec3f(-1.f, 3.f, 5.f));
        const Vec3f before = camera.position();
        camera.mouseDrag(Vec2f(0.f));
        Check((camera.position() - before).Length() < 1e-5f, "Orbit conversion must preserve negative quadrants and translated origin");
        Vec3f f, u, r;
        camera.directions(f, u, r);
        Check(u.Dot(WORLD_Y) > 0.f, "Camera up must point upward");

        CustomPtCloud bunny;
        const auto bunnyPath = std::filesystem::path(argv[1]) / "bunny-points.obj";
        Check(bunny.LoadFromFileObj(bunnyPath.string().c_str()), "Stanford bunny must load and estimate all areas");
        Check(bunny.HasPrecomputedAreas(), "Bundled bunny must use its precomputed mesh areas");
        Check(std::abs(std::accumulate(bunny.Areas().begin(),bunny.Areas().end(),0.)-.057128783477919554)<1e-8,
              "Bunny weights must conserve the source mesh's surface area");
        const double beforePotential = DipolePotential(bunny, Vec3f(10.f));
        bunny.Normalize();
        for (const auto& p : bunny.Positions()) Check(p.Length() <= 1.00001f, "Normalized cloud must fit unit sphere");
        std::cout << "All point-cloud regressions passed; sphere area=" << totalArea
                  << "; bunny points=" << bunny.Positions().size() << "; far dipole potential=" << beforePotential << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
