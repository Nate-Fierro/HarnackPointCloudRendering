#include <iostream>
#include <numeric>
#include "point_cloud.h"
#include "viewer.h"
#include "screenshot.h"

int main(int argc, char** argv) {
    try {
        std::filesystem::path input = std::filesystem::path(HARNACK_SOURCE_DIR) / "data/bunny-points.obj";
        std::filesystem::path screenshot;
        bool inspectOnly = false, estimateAreas = false;
        int width = 320, height = 320;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: harnack_debug [points.obj] [--validate] [--estimate-areas] [--screenshot output.ppm] [--size width height]\n";
                return 0;
            } else if (arg == "--validate") inspectOnly = true;
            else if (arg == "--estimate-areas") estimateAreas = true;
            else if (arg == "--screenshot" && i + 1 < argc) screenshot = argv[++i];
            else if (arg == "--size" && i + 2 < argc) {
                width = std::stoi(argv[++i]);
                height = std::stoi(argv[++i]);
            } else if (!arg.empty() && arg[0] == '-') throw std::runtime_error("Unknown/incomplete option: " + arg);
            else input = arg;
        }
        CustomPtCloud cloud;
        if (!cloud.LoadFromFileObj(input.string().c_str(), &std::cout, !estimateAreas)) return 1;
        cloud.Normalize();
        std::cout << cloud.Positions().size() << " points; normalized area="
                  << std::accumulate(cloud.Areas().begin(), cloud.Areas().end(), 0.)
                  << "; " << (cloud.HasPrecomputedAreas() ? "precomputed areas" : "local estimates") << '\n';
        if (inspectOnly) return 0;
        if (width <= 0 || height <= 0) throw std::runtime_error("Image dimensions must be positive");
        viewer::Initialize(argv[0], cloud, width, height, screenshot.empty());
        if (screenshot.empty()) viewer::Run();
        else {
            viewer::DrawFrame();
            SaveScreenshot(screenshot, width, height);
            viewer::Close();
            std::cout << "Saved " << screenshot << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        viewer::Close();
        return 1;
    }
}
