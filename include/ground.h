#pragma once
#include<glm/glm.hpp>

float groundVertices[] = {
    // positions          // colors
    -5.0f, -0.5f, -5.0f,  0.58f, 0.29f, 0.2f,
     5.0f, -0.5f, -5.0f,  0.58f, 0.29f, 0.2f,
     5.0f, -0.5f,  5.0f,  0.58f, 0.29f, 0.2f,
     5.0f, -0.5f,  5.0f,  0.58f, 0.29f, 0.2f,
    -5.0f, -0.5f,  5.0f,  0.58f, 0.29f, 0.2f,
    -5.0f, -0.5f, -5.0f,  0.58f, 0.29f, 0.2f,
};

btVector3 groundPositions[] = {
    btVector3(0.0f, 0.0f,  0.0f)
};
