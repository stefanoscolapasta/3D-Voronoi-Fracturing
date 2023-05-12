#pragma once
#include <bullet/LinearMath/btVector3.h>
#include <GLFW/glfw3.h>
#include "camera.h"
#include "tetrahedron.h"

#ifndef UTILS_H
#define UTILS_H
#define N 4
#define SCR_WIDTH 800
#define SCR_HEIGHT  600



//OPENGL UTILS

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);
GLFWwindow* createWindow();
Camera getCamera();
void setFrame(float deltaTime_, float lastFrame_);
float getDeltaTime();
float getLastFrame();

//MATHS

//RETURN VALUE:
//>0 -> p is above the plane defined by a, b,c
//<0 -> p is under the plane defined by a, b,c
// = 0-> p is on the plane defined by a, b,c
btVector3 getSphereCenter(std::set<btVector3> points);
glm::vec3 intersection(glm::vec3 normal1, glm::vec3 point1, glm::vec3 normal2, glm::vec3 point2);
int orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p);
float determinantOfMatrix(float matrix[N][N], int n);
void subMatrix(float mat[N][N], float temp[N][N], int p, int q, int n);
btVector3 getTetrahedronCenter(Tetrahedron tetrahedron);
bool isPointInsideSphere(Tetrahedron tetrahedron, btVector3 P);


//TETRAS
bool isFacetInTetrahedron(const Tetrahedron& t, const TriangleFacet& f);
bool areTetrasEqual(Tetrahedron t1, Tetrahedron t2);
bool areTriangleFacetsEqual(const TriangleFacet& f1, const TriangleFacet& f2);
bool isPointInsideTetrahedron(Tetrahedron tetrahedron, btVector3  point);
#endif
