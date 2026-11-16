#include <GL/glew.h>
#include <fstream>
#include <stdexcept>
#include <vector>
#include "screenshot.h"

void SaveScreenshot(const std::filesystem::path& path, int width, int height) {
    std::vector<unsigned char> pixels(size_t(width) * height * 3);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glFinish();
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    std::ofstream output(path, std::ios::binary);
    output << "P6\n" << width << ' ' << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y)
        output.write(reinterpret_cast<const char*>(pixels.data() + size_t(y) * width * 3), std::streamsize(width) * 3);
    if (!output) throw std::runtime_error("Cannot write screenshot: " + path.string());
}
