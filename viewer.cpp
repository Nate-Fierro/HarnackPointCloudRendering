#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cyGL.h>
#include <filesystem>
#include <stdexcept>
#include "camera.h"
#include "point_cloud.h"
#include "viewer.h"

namespace viewer {
namespace {
GLSLProgram program;
OrbitCamera orbitCam;
GLuint pointBuffer = 0, vao = 0;
int window = 0;
Vec2u resolution(640, 640);
Vec2i previousMouse(0);
bool dragging = false;

// std430: position/area, followed by normal/padding.
struct alignas(16) CloudPoint {
    Vec3f pos;
    float area;
    Vec3f norm;
    float padding = 0.f;
};

enum Uniforms : uint { CAMERA_POS, CAMERA_F, CAMERA_U, CAMERA_R, FOV, RES };

void updateCameraUniform() {
    Vec3f f, u, r;
    orbitCam.directions(f, u, r);
    const Vec3f p = orbitCam.position();
    program.Bind();
    program.SetUniform3(CAMERA_POS, &p.x);
    program.SetUniform3(CAMERA_F, &f.x);
    program.SetUniform3(CAMERA_U, &u.x);
    program.SetUniform3(CAMERA_R, &r.x);
}

void display() {
    DrawFrame();
    glutSwapBuffers();
}

void resize(int width, int height) {
    resolution.Set(uint(std::max(width, 1)), uint(std::max(height, 1)));
    glViewport(0, 0, GLsizei(resolution.x), GLsizei(resolution.y));
    program.Bind();
    program.SetUniform(RES, resolution.x, resolution.y);
    glutPostRedisplay();
}

void mouseClick(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) dragging = state == GLUT_DOWN;
    previousMouse.Set(x, y);
}

void mouseDrag(int x, int y) {
    if (!dragging) return;
    orbitCam.mouseDrag(Vec2f(float(x - previousMouse.x), float(previousMouse.y - y)));
    previousMouse.Set(x, y);
    updateCameraUniform();
    glutPostRedisplay();
}

void mouseWheel(int, int direction, int, int) {
    orbitCam.mouseWheel(0.f, float(direction) * 0.15f);
    updateCameraUniform();
    glutPostRedisplay();
}

void keyPressed(unsigned char key, int, int) {
    if (key == 27 || key == 'q') glutLeaveMainLoop();
    if (key == 'r') {
        orbitCam.setPosition(Vec3f(0.0f, 0.25f, 3.0f));
        updateCameraUniform();
        glutPostRedisplay();
    }
}
}

void Initialize(const char* executable, const CustomPtCloud& cloud, int width, int height, bool visible) {
    int argc = 1;
    char* argv[] = {const_cast<char*>(executable), nullptr};
    glutInit(&argc, argv);
    glutInitContextVersion(4, 5);
    glutInitContextProfile(GLUT_CORE_PROFILE);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width, height);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    window = glutCreateWindow("Harnack tracing | oriented point cloud");
    if (!visible) glutHideWindow();
    glewExperimental = GL_TRUE;
    const GLenum status = glewInit();
    if (status != GLEW_OK) throw std::runtime_error(reinterpret_cast<const char*>(glewGetErrorString(status)));

    const auto exeDir = std::filesystem::absolute(executable).parent_path();
    auto shaderPath = [&](const char* name) {
        const auto adjacent = exeDir / name;
        return std::filesystem::exists(adjacent) ? adjacent : std::filesystem::path(HARNACK_SOURCE_DIR) / name;
    };
    const auto vertex = shaderPath("ScreenQuad.vert"), fragment = shaderPath("Harnack_PtCld.frag");
    if (!program.BuildFiles(vertex.string().c_str(), fragment.string().c_str()))
        throw std::runtime_error("Cannot compile/link renderer shaders");
    program.RegisterUniform(CAMERA_POS, "cam.pos");
    program.RegisterUniform(CAMERA_F, "cam.f");
    program.RegisterUniform(CAMERA_U, "cam.u");
    program.RegisterUniform(CAMERA_R, "cam.r");
    program.RegisterUniform(FOV, "fov");
    program.RegisterUniform(RES, "res");
    orbitCam.setPosition(Vec3f(0.0f, 0.25f, 3.0f));
    updateCameraUniform();
    program.SetUniform(FOV, degreesToRadians(45.f));
    resize(width, height);

    std::vector<CloudPoint> points;
    points.reserve(cloud.Positions().size());
    for (size_t i = 0; i < cloud.Positions().size(); ++i)
        points.push_back({cloud.Positions()[i], cloud.Areas()[i], cloud.Normals()[i], 0.f});
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &pointBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pointBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, GLsizeiptr(points.size() * sizeof(CloudPoint)), points.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pointBuffer);
}

void DrawFrame() {
    program.Bind();
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void Run() {
    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseDrag);
    glutMouseWheelFunc(mouseWheel);
    glutKeyboardFunc(keyPressed);
    glutMainLoop();
}

void Close() {
    if (window) glutDestroyWindow(window);
    window = 0;
}
}
