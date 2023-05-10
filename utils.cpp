#include"utils.h"

//OPENGL UTILS

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;


// camera
Camera camera(glm::vec3(0.0f, 0.0f, 15.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

Camera getCamera() {
    return camera;
}

float getLastFrame() {
    return lastFrame;
}

float getDeltaTime() {
    return deltaTime;
}
void setFrame(float deltaTime_, float lastFrame_){
    deltaTime = deltaTime_;
    lastFrame = lastFrame_;
}
GLFWwindow* createWindow() {
    GLFWwindow*window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    return window;
}
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}


// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}




//MATHS

int Orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p) {
    float mat[N][N] = {
        a.getX(), a.getY(), a.getZ(), 1,
        b.getX(), b.getY(), b.getZ(), 1,
        c.getX(), c.getY(), c.getZ(), 1,
        p.getX(), p.getY(), p.getZ(), 1
    };
    return determinantOfMatrix(mat, N);

}


float determinantOfMatrix(float matrix[N][N], int n) {
    float determinant = 0.0;
    if (n == 1) {
        return matrix[0][0];
    }
    if (n == 2) {
        return (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
    }
    float temp[N][N], sign = 1;
    for (int i = 0; i < n; i++) {
        subMatrix(matrix, temp, 0, i, n);
        determinant += sign * matrix[0][i] * determinantOfMatrix(temp, n - 1);
        sign = -sign;
    }
    return determinant;
}

void subMatrix(float mat[N][N], float temp[N][N], int p, int q, int n) {
    int i = 0, j = 0;
    // filling the sub matrix
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            // skipping if the current row or column is not equal to the current
            // element row and column
            if (row != p && col != q) {
                temp[i][j++] = mat[row][col];
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

#include <glm/glm.hpp>

glm::vec3 convertToVec3(btVector3 vec) {
    glm::vec3 vector = glm::vec3(vec.getX(), vec.getY(), vec.getZ());
    return vector;
}

btVector3 getSphereCenter(btVector3 points[]) {
    glm::vec3 P1 = convertToVec3(points[0]);
    glm::vec3 P2 = convertToVec3(points[1]);
    glm::vec3 P3 = convertToVec3(points[2]);
    glm::vec3 P4 = convertToVec3(points[3]);

    glm::vec3 midpoint1 = 0.5f * (P1 + P2);
    glm::vec3 midpoint2 = 0.5f * (P3 + P4);
    glm::vec3 normal1 = glm::normalize(glm::cross(P2 - P1, midpoint1 - P1));
    glm::vec3 normal2 = glm::normalize(glm::cross(P4 - P3, midpoint2 - P3));
    glm::vec3 sphereCenter = intersection(normal1, midpoint1, normal2, midpoint2);

    return btVector3(sphereCenter.x, sphereCenter.y, sphereCenter.z);
}

//bool isPointInsideSphere(Tetrahedron tetrahedron, btVector3 P) {
//    glm::vec3 center = getSphereCenter(//vertices of tetra 
//    );
//
//    //vertices conversion
//    //struttura dati provvisoria
//    glm::vec3 P1 = convertToVec3(tetrahedron.vertices[0]);
//    glm::vec3 P2 = convertToVec3(tetrahedron.vertices[1]);
//    glm::vec3 P3 = convertToVec3(tetrahedron.vertices[2]);
//    glm::vec3 P4 = convertToVec3(tetrahedron.vertices[3]);
//
//
//    glm::vec3 point = convertToVec3(P);
//    float radius = glm::length(center - P1);
//    radius = glm::max(radius, glm::length(center - P2));
//    radius = glm::max(radius, glm::length(center - P3));
//    radius = glm::max(radius, glm::length(center - P4));
//    float distance = glm::distance(point, center);
//
//    float distance = glm::distance(point, center);
//    bool isInside = distance <= radius;
//    return isInside;
//}

bool isPointInsideTetrahedron(Tetrahedron tetrahedron, btVector3  point) {
    // Get the center and radius of the circumsphere of the tetrahedron
    btVector3 center = getSphereCenter(tetrahedron.allSingularVertices);
    float radius = (tetrahedron.allSingularVertices[0] - center).length();

    // Check if the point is inside the circumsphere
    float distance = (point - center).length();
    if (distance > radius) {
        return false;
    }

    // Check if the point is inside each facet of the tetrahedron
    for (int i = 0; i < 4; i++) {
        if (!isPointInsideFacet(tetrahedron.facets[i], point)) {
            return false;
        }
    }

    // The point is inside the tetrahedron
    return true;
}

bool isPointInsideFacet(TriangleFacet facet, btVector3 point) {
    // Compute the barycentric coordinates of the point with respect to the facet
    btVector3 v0 = facet.vertices[0];
    btVector3 v1 = facet.vertices[1];
    btVector3 v2 = facet.vertices[2];
    btVector3 w = point - v0;
    btVector3 u = v1 - v0;
    btVector3 v = v2 - v0;
    float uu = u.dot(u);
    float uv = u.dot(v);
    float vv = v.dot(v);
    float wu = w.dot(u);
    float wv = w.dot(v);
    float denom = uv * uv - uu * vv;
    float s = (uv * wv - vv * wu) / denom;
    float t = (uv * wu - uu * wv) / denom;
    float u1 = 1 - s - t;

    // Check if the barycentric coordinates are between 0 and 1
    if (s >= 0 && t >= 0 && u1 >= 0) {
        return true;
    }
    else {
        return false;
    }
}


bool areTetrasEqual(Tetrahedron t1, Tetrahedron t2) {
    for (int i = 0; i < 4; i++) {
        if (t1.allSingularVertices[i] != t2.allSingularVertices[i]) {
            return false;
        }
    }
    return true;
}



glm::vec3 intersection(glm::vec3 normal1, glm::vec3 point1, glm::vec3 normal2, glm::vec3 point2) {
    glm::vec3 dir = glm::normalize(glm::cross(normal1, normal2));
    float d1 = glm::dot(normal1, point1);
    float d2 = glm::dot(normal2, point2);
    float d3 = glm::dot(dir, point1 - point2);
    float t = (d1 - d2) / d3;
    return point1 + t * dir;
}



