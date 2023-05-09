#pragma once
#include <bullet/LinearMath/btVector3.h>
#include <GLFW/glfw3.h>
#include "camera.h"
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

//RETURN VALUE:
//>0 -> p is above the plane defined by a, b,c
//<0 -> p is under the plane defined by a, b,c
// = 0-> p is on the plane defined by a, b,c
btVector3 getSphereCenter(btVector3 p1, btVector3 p2, btVector3 p3, btVector3 p4);
glm::vec3 intersection(glm::vec3 normal1, glm::vec3 point1, glm::vec3 normal2, glm::vec3 point2);
int Orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p);
float determinantOfMatrix(float matrix[N][N], int n);
void subMatrix(float mat[N][N], float temp[N][N], int p, int q, int n);

#endif
