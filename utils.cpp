#include "incl/utils.h"
#include <algorithm>


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
//determines if a point p is over, under or lies on a plane defined by three points a, b and c
//>0 -> p is above the plane defined by a, b,c
//<0 -> p is under the plane defined by a, b,c
// = 0-> p is on the plane defined by a, b,c
int orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p) {
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

glm::vec3 intersection(glm::vec3 normal1, glm::vec3 point1, glm::vec3 normal2, glm::vec3 point2) {
    glm::vec3 dir = glm::normalize(glm::cross(normal1, normal2));
    float d1 = glm::dot(normal1, point1);
    float d2 = glm::dot(normal2, point2);
    float d3 = glm::dot(dir, point1 - point2);
    float t = (d1 - d2) / d3;
    return point1 + t * dir;
}





glm::vec3 convertToVec3(btVector3 vec) {
    glm::vec3 vector = glm::vec3(vec.getX(), vec.getY(), vec.getZ());
    return vector;
}


btVector3 getTetrahedronCenter(Tetrahedron tetrahedron) {
    btVector3 center(0, 0, 0);

    // Calculate the average of the four vertices
    for (auto& v : tetrahedron.allSingularVertices) {
        center += v;
    }
    center /= 4.0;

    return center;
}

btVector3 getSphereCenter(std::set<btVector3> points) {
    std::vector<glm::vec3> points_vec3;
    for (auto point : points) {
        points_vec3.push_back(convertToVec3(point));
    }

    // Find midpoints of two line segments
    glm::vec3 midpoint1 = (points_vec3[0] + points_vec3[1]) / 2.0f;
    glm::vec3 midpoint2 = (points_vec3[1] + points_vec3[2]) / 2.0f;

    // Find direction vectors of line segments
    glm::vec3 direction1 = points_vec3[1] - points_vec3[0];
    glm::vec3 direction2 = points_vec3[2] - points_vec3[1];

    // Find normal vectors of perpendicular bisectors
    glm::vec3 normal1 = glm::normalize(glm::cross(direction1, glm::vec3(0, 1, 0)));
    glm::vec3 normal2 = glm::normalize(glm::cross(direction2, glm::vec3(0, 1, 0)));

    // Find intersection point of perpendicular bisectors
    glm::vec3 center = glm::cross(normal1, normal2);
    center = midpoint1 + ((glm::dot(normal2, normal2) * glm::cross(normal1, center)) - (glm::dot(normal1, normal1) * glm::cross(normal2, center))) / (2.0f * glm::dot(normal1, normal2));

    // Return center of sphere
    return btVector3(center.x, center.y, center.z);
}


bool isPointInsideSphere(Tetrahedron tetrahedron, btVector3 p) {
    btVector3 center_bt = getSphereCenter(tetrahedron.allSingularVertices);
    glm::vec3 center = convertToVec3(center_bt);
    std::vector<glm::vec3> tetra_vertices;
    //vertices conversion
    int i = 0;
    for (std::set<btVector3>::iterator v = tetrahedron.allSingularVertices.begin();
        v != tetrahedron.allSingularVertices.end(); v++) {
        btVector3 vertex = *v;
        tetra_vertices[i] = convertToVec3(vertex);
        i++;
    }

    glm::vec3 point = convertToVec3(p);
    float radius = glm::length(center - tetra_vertices[0]);
    radius = glm::max(radius, glm::length(center - tetra_vertices[1]));
    radius = glm::max(radius, glm::length(center - tetra_vertices[2]));
    radius = glm::max(radius, glm::length(center - tetra_vertices[3]));
    float distance = glm::distance(point, center);
    bool isInside = distance <= radius;
    return isInside;
}

bool isPointInsideTetrahedron(Tetrahedron tetrahedron, btVector3  point) {
    // Iterate over each face of the tetrahedron
    for (auto& facet : tetrahedron.facets) {
        // Get the three vertices of the face
        const btVector3& v0 = facet.vertices[0];
        const btVector3& v1 = facet.vertices[1];
        const btVector3& v2 = facet.vertices[2];

        // Calculate the normal vector of the face
        btVector3 normal = (v1 - v0).cross(v2 - v0).normalized();

        // Calculate the distance from the origin to the face
        float distance = -v0.dot(normal);

        // Calculate the distance from the point to the plane defined by the face
        float pointDistance = point.dot(normal) + distance;

        // If the point is on the opposite side of the plane from the tetrahedron, it is outside
        if (pointDistance < 0) {
            return false;
        }
    }

    // If the point is on the same side of all four faces, it is inside
    return true;
}

bool isFacetInTetrahedron(const Tetrahedron& t, const TriangleFacet& f) {
    for (const auto& tf : t.facets) {
        if (areTriangleFacetsEqual(tf, f)) {
            return true;
        }
    }
    return false;
}

bool areTetrasEqual(Tetrahedron t1, Tetrahedron t2) {

    // Compare the vertices of each facet
    for (int i = 0; i < t1.facets.size(); i++) {
        TriangleFacet& f1 = t1.facets[i];
        TriangleFacet& f2 = t2.facets[i];

        for (int j = 0; j < f1.vertices.size(); j++) {
            if (f1.vertices[j] != f2.vertices[j]) {
                return false;
            }
        }
    }
}


bool areTriangleFacetsEqual(const TriangleFacet& f1, const TriangleFacet& f2) {
    // Check if each vertex is equal
    std::vector<btVector3> sortedVerticesF1 = f1.vertices;
    std::vector<btVector3> sortedVerticesF2 = f2.vertices;
    btVector3Comparator comparator;
    std::sort(sortedVerticesF1.begin(), sortedVerticesF1.end(), comparator);
    std::sort(sortedVerticesF2.begin(), sortedVerticesF2.end(), comparator);

    for (int i = 0; i < sortedVerticesF1.size(); i++) {
        if (sortedVerticesF1[i] != sortedVerticesF2[i]) {
            return false;
        }
    }

    // The facets are equal
    return true;
}

TriangleFacet findSharedFacet(Tetrahedron t1, Tetrahedron t2) {
    for (auto f1 : t1.facets) {
        for (auto f2 : t2.facets) {
            if (areTriangleFacetsEqual(f1,f2)) {
                // The facets match, so return f1
                return f1;
            }
        }
    }

}

std::vector<Tetrahedron> getTetrasIncidentToEdge(btVector3 v1, btVector3 v2, std::vector<Tetrahedron> tetrahedra) {
    std::vector<Tetrahedron> result;

    for (Tetrahedron tetrahedron : tetrahedra) {
        bool foundv1 = false;
        bool foundv2 = false;
        for (TriangleFacet facet : tetrahedron.facets) {
            for (btVector3 v : facet.vertices) {
                if (v == v1) {
                    foundv1 = true;
                }
                else if (v == v2) {
                    foundv2 = true;
                }
            }
            if (foundv1 && foundv2) {
                for (const btVector3& v : facet.vertices) {
                    if (v != v1 && v != v2) {
                        result.push_back(tetrahedron);
                        break;
                    }
                }
                break;
            }
            else {
                foundv1 = false;
                foundv2 = false;
            }
        }
    }

    return result;
}


// Function to convert Vector to Set
std::set<btVector3> convertToSet(std::vector<btVector3> v)
{
    std::set<btVector3> s;
    for (btVector3 x : v) {
        s.insert(x);
    }
    return s;
}


std::vector<float> convertVertexVectorToFlatFloatArr(std::vector<Vertex> allVertices) {
    std::vector<float> allVerticesAsFloatArr;
    for (auto& vertice : allVertices) {
        std::vector<float> vectorComponents = generateVerticesArrayFromVertex(vertice);
        allVerticesAsFloatArr.insert(allVerticesAsFloatArr.end(), vectorComponents.begin(), vectorComponents.end());
    }
    return allVerticesAsFloatArr;
}

void vectorToFloatArray(const std::vector<float>& vec, float arr[]) {
    for (size_t i = 0; i < vec.size(); i++) {
        arr[i] = vec[i];
    }
}

std::vector<float> generateVerticesArrayFromVertex(Vertex v) {
    return { (float)v.Position.x, (float)v.Position.y, (float)v.Position.z };
}




