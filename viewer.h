#pragma once

class CustomPtCloud;

namespace viewer {
void Initialize(const char* executable, const CustomPtCloud& cloud,
                int width = 640, int height = 640, bool visible = true);
void DrawFrame();
void Run();
void Close();
}
