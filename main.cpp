#include <iostream>
#include "point_cloud.h"
#include "viewer.h"

int main(int argc, char** argv) {

    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: harnack [points.obj]\n";
        return argc > 2 ? 1 : 0;
    }
    const char* input = argc == 2 ? argv[1] : HARNACK_SOURCE_DIR "/data/bunny-points.obj";
    CustomPtCloud cloud;
    if (!cloud.LoadFromFileObj(input)) return 1;
    cloud.Normalize();

    try {
        viewer::Initialize(argv[0], cloud);
        viewer::Run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
