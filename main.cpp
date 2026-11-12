#define _CRT_SECURE_NO_WARNINGS

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <iostream>
#include <functional>
#include <unordered_map>
#include <string>
#include <numbers>
#include <vector>

#include <cyGL.h>
#include <cyPointCloud.h>

#include "helper.h"
#include "camera.h"
#include "point_cloud.h"

// Callback functions
void display();
void mouseDragMotion(int x, int y);
void mouseClick(int button, int state, int x, int y);
void resize(int width, int height);
void arrowKeyPressed(int key, int x, int y);
void characterKeyPressed(unsigned char key, int x, int y);

// Helper functions (see bottom of file)
void updateCameraUniform();

struct Mouse {
	struct Drag {
		Vec2i prevPos;
	} drag;
} mouse;

// Shader Programs
struct Programs {
	GLSLProgram harnack;
} prog;

struct Buffers {
	GLuint ptCloud;
} buf;

GLuint vao;

OrbitCamera orbitCam;

struct Transform {
	float scale;
	Vec3f pos;
} trans;

struct CloudPointStd430 {
	Vec3f pos;
	float area;
	Vec3f norm;
	float padding;
};

Vec2u res;

struct Camera {
	Vec3f pos;
	Vec3f f;
	Vec3f u;
	Vec3f r;
};

enum Uniforms : uint { CAMERA_POS, CAMERA_F, CAMERA_U, CAMERA_R, FOV, RES, TRANSFORM };

int main(int argc, char* argv[]) {
	// Initialize GLUT and GLEW
	glutInit(&argc, argv);
	glutInitContextVersion(4, 5);
	glutInitContextProfile(GLUT_CORE_PROFILE);	

	//glutInitContextFlags(GLUT_DEBUG);

	glutInitWindowSize(1024, 1024);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("Harnack Tracing for Point Clouds | Nate Fierro");

	glewInit();

	const unsigned char* version = glGetString(GL_VERSION);
	std::cout << version << std::endl;

	orbitCam.setPosition(Vec3f(1.0f, 0.0f, 0.0f));

	bool success = prog.harnack.BuildFiles("ScreenQuad.vert", "Harnack_PtCld.frag");
	if (!success) {
		std::cout << "ERROR: Couldn't compile and/or link shaders" << std::endl;
		return -1;
	}

	prog.harnack.RegisterUniform(CAMERA_POS, "cam.pos");
	prog.harnack.RegisterUniform(CAMERA_F, "cam.f");
	prog.harnack.RegisterUniform(CAMERA_U, "cam.u");
	prog.harnack.RegisterUniform(CAMERA_R, "cam.r");
	prog.harnack.RegisterUniform(RES, "res");
	prog.harnack.RegisterUniform(FOV, "fov");
	//prog.harnack.RegisterUniform(TRANSFORM, "ptCldTrans");

	updateCameraUniform();

	prog.harnack.Bind();
	prog.harnack.SetUniform(FOV, degreesToRadians(70.0f));

	// Parse file
	CustomPtCloud ptCldIO;

	if (argc < 2) { std::cout << "Expected a file name" << std::endl; return -1; }

	ptCldIO.LoadFromFileObj(argv[1]);

	if (!success) { return -1; }

	if (ptCldIO.Positions().size() == 0 || ptCldIO.Normals().size() < ptCldIO.Positions().size()) {
		std::cout << "Must have a normal for each point " << argv[1] << " " << std::endl; return -1;
	}

	uint size = uint(ptCldIO.Positions().size());

	std::vector<CloudPointStd430> ptCloud;

	for (uint i = 0; i < size; ++i) {
		ptCloud.emplace_back();
		ptCloud.back().pos = ptCldIO.Positions()[i];
		ptCloud.back().norm = ptCldIO.Normals()[i];
		ptCloud.back().area = ptCldIO.Areas()[i];
	}

	// TODO: Center point cloud on origin

	// Build VAO and Point Cloud SSBO
	glGenVertexArrays(1, &vao);

	glGenBuffers(1, &buf.ptCloud);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, buf.ptCloud);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf.ptCloud);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(CloudPointStd430) * ptCloud.size(), ptCloud.data(), GL_STATIC_DRAW);
	
	// Register callback functions
	glutDisplayFunc(display);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseDragMotion);
	glutReshapeFunc(resize);
	glutSpecialFunc(arrowKeyPressed);
	glutKeyboardFunc(characterKeyPressed);

	// Final preparations before entering main loop
	glClearColor(0, 0, 0, 1);

	glutMainLoop();

	return 0;
}

/// <summary> Callback function for rendering a new frame to the window. </summary>
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	prog.harnack.Bind();
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	
	glutSwapBuffers();
}

/// <summary>
/// Callback function for capturing mouse movements while
///	a mouse button is held down. 
/// </summary>
/// <param name="x"> The horizontal coordinate of the mouse </param>
/// <param name="y"> The vertical coordinate of the mouse (down is the positive direction) </param>
void mouseDragMotion(int x, int y) {
	Vec2f change;
	change.y = mouse.drag.prevPos.y - y;
	change.x = x - mouse.drag.prevPos.x;

	mouse.drag.prevPos.Set(x, y);

	orbitCam.mouseDrag(change);

	updateCameraUniform();

	glutPostRedisplay();
}

/// <summary> Callback function for capturing mouse clicks. </summary>
/// <param name="button">
/// The button clicked. One of the following:
///		1. GLUT_LEFT_BUTTON
///		2. GLUT_RIGHT_BUTTON
///		3. GLUT_MIDDLE_BUTTON
/// </param>
/// <param name="state"> Either GLUT_UP or GLUT_DOWN </param>
/// <param name="x"> The horizontal coordinate of the mouse </param>
/// <param name="y"> The vertical coordinate of the mouse (down is the positive direction) </param>
void mouseClick(int button, int state, int x, int y) {
	if (state == GLUT_DOWN) {
		mouse.drag.prevPos.Set(x, y);
	}
}

void arrowKeyPressed(int key, int x, int y) {
	return;
}

void characterKeyPressed(unsigned char key, int x, int y) {
	return;
}

/// <summary>
/// Callback function that is called prior to displaying the first
/// frame and when the window is resized.
/// </summary>
/// <param name="width"> The width of the window. </param>
/// <param name="height"> The height of the window. </param>
void resize(int width, int height) {
	glViewport(0, 0, width, height);

	res.Set(uint(width), uint(height));

	prog.harnack.Bind();
	prog.harnack.SetUniform(RES, res.x, res.y);

	glutPostRedisplay();
}

void updateCameraUniform() {
	Camera cam;
	orbitCam.directions(cam.f, cam.u, cam.r);
	cam.pos = orbitCam.position();

	prog.harnack.Bind();
	prog.harnack.SetUniform3(CAMERA_POS, &cam.pos.x);
	prog.harnack.SetUniform3(CAMERA_F, &cam.f.x);
	prog.harnack.SetUniform3(CAMERA_U, &cam.u.x);
	prog.harnack.SetUniform3(CAMERA_R, &cam.r.x);
}
