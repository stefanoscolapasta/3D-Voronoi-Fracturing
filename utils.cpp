#include "incl/utils.h"
#include <algorithm>
#include<math.h> 
//OPENGL UTILS
bool startSimulation = false;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;


// camera
Camera camera(glm::vec3(0.0f, 0.0f, 15.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
bool accelerationMultiplier = false;

bool isSimulationStarted() {
    return startSimulation;
}

Camera getCamera() {
    return camera;
}

float getLastFrame() {
    return lastFrame;
}

float getDeltaTime() {
    return deltaTime;
}
void setFrame(float deltaTime_, float lastFrame_) {
    deltaTime = deltaTime_;
    lastFrame = lastFrame_;
}
GLFWwindow* createWindow() {
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    return window;
}
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime, accelerationMultiplier);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime, accelerationMultiplier);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime, accelerationMultiplier);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime, accelerationMultiplier);

    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
        startSimulation = true;
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_RELEASE)
        startSimulation = false;

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        accelerationMultiplier = true;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE)
        accelerationMultiplier = false;
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

std::vector<btVector3>  sortFacetVerticesCounterClockwise(std::vector<btVector3> vertices) {
    std::vector<btVector3> orderedVertices = vertices;
    // Get the vertices
    btVector3& a = orderedVertices[0];
    btVector3& b = orderedVertices[1];
    btVector3& c = orderedVertices[2];

    // Calculate the vectors
    btVector3 edge1 = b - a;
    btVector3 edge2 = c - a;

    // Calculate the cross product magnitude
    float signedArea = edge1.cross(edge2).length();

    // Check the winding order and swap vertices if necessary
    if (signedArea < 0) {
        // Swap b and c
        std::swap(orderedVertices[1], orderedVertices[2]);
    }
    return orderedVertices;

}


//MATHS
//determines if a point p is over, under or lies on a plane defined by three points a, b and c
//>0 -> p is above the plane defined by a, b,c
//<0 -> p is under the plane defined by a, b,c
// = 0-> p is on the plane defined by a, b,c
int orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p) {
    btVector3 facetCenter = (a + b + c) / 3;
    btVector3 v = p - facetCenter;

    btVector3 edge1 = b - a;
    btVector3 edge2 = c - a;

    btVector3 normal = edge1.cross(edge2);
    normal.normalize();

    float vDotPlaneNormal = v.dot(normal);
    if (vDotPlaneNormal < 0)
        return -1;
    else if (vDotPlaneNormal > 0)
        return 1;
    return 0;
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


//CONVERSIONS


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

btVector3 getSphereCenter(std::set<btVector3, btVector3Comparator> points) {
#define U(a,b,c,d,e,f,g,h) (a.z - b.z)*(c.x*d.y - d.x*c.y) - (e.z - f.z)*(g.x*h.y - h.x*g.y)
#define D(x,y,a,b,c) (a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y))
#define E(x,y) ((ra*D(x,y,b,c,d) - rb*D(x,y,c,d,a) + rc*D(x,y,d,a,b) - rd*D(x,y,a,b,c)) / uvw)
    std::vector<glm::vec3> points_vec3;
    for (auto point : points) {
        points_vec3.push_back(glm::vec3(point.getX(), point.getY(), point.getZ()));
    }

    glm::vec3 a = points_vec3[0];
    glm::vec3 b = points_vec3[1];
    glm::vec3 c = points_vec3[2];
    glm::vec3 d = points_vec3[3];

    double u = U(a, b, c, d, b, c, d, a);
    double v = U(c, d, a, b, d, a, b, c);
    double w = U(a, c, d, b, b, d, a, c);
    double uvw = 2 * (u + v + w);
    if (uvw == 0.0) {
        // Oops.  The points are coplanar.
    }
    auto sq = [](glm::vec3 p) { return p.x * p.x + p.y * p.y + p.z * p.z; };
    double ra = sq(a);
    double rb = sq(b);
    double rc = sq(c);
    double rd = sq(d);
    double x0 = E(y, z);
    double y0 = E(z, x);
    double z0 = E(x, y);
    return btVector3(x0, y0, z0);

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

bool isPointInsideTetrahedron(Tetrahedron tetrahedron, btVector3 point) {

    std::set<btVector3, btVector3Comparator> vertices = tetrahedron.allSingularVertices;
    btVector3 v0 = *std::next(vertices.begin(), 0);
    btVector3 v1 = *std::next(vertices.begin(), 1);
    btVector3 v2 = *std::next(vertices.begin(), 2);
    btVector3 v3 = *std::next(vertices.begin(), 3);

    glm::vec3 v0_vec3 = glm::vec3(v0.getX(), v0.getY(), v0.getZ());
    glm::vec3 v1_vec3 = glm::vec3(v1.getX(), v1.getY(), v1.getZ());
    glm::vec3 v2_vec3 = glm::vec3(v2.getX(), v2.getY(), v2.getZ());
    glm::vec3 v3_vec3 = glm::vec3(v3.getX(), v3.getY(), v3.getZ());
    glm::vec3 p = glm::vec3(point.getX(), point.getY(), point.getZ());

    return SameSide(v0_vec3, v1_vec3, v2_vec3, v3_vec3, p) &&
        SameSide(v1_vec3, v2_vec3, v3_vec3, v0_vec3, p) &&
        SameSide(v2_vec3, v3_vec3, v0_vec3, v1_vec3, p) &&
        SameSide(v3_vec3, v0_vec3, v1_vec3, v2_vec3, p);
}

bool SameSide(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 p)
{
    glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
    float dotV3 = glm::dot(normal, v3 - v0);
    float dotP = glm::dot(normal, p - v0);
    return signbit(dotV3) == signbit(dotP);
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
    for (auto &f1 : t1.facets) {
        for (auto &f2 : t2.facets) {
            if (areTriangleFacetsEqual(f1, f2)) {
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

// Function to calculate the bounding box of a cubic mesh
void CalculateBoundingBox(std::vector<btVector3> meshVertices, btVector3& minCoords, btVector3& maxCoords) {
    minCoords = btVector3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    maxCoords = btVector3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (const auto& vertex : meshVertices) {
        minCoords.setMin(vertex);
        maxCoords.setMax(vertex);
    }
}


// Function to create a tetrahedron that includes the mesh
Tetrahedron CreateTetrahedronAroundShape(std::vector<btVector3> meshVertices, glm::vec3 meshColor) {
    Tetrahedron tetrahedron;

    // Calculate the bounding box of the shape
    btVector3 minCoords, maxCoords;
    CalculateBoundingBox(meshVertices, minCoords, maxCoords);

    // Determine the center of the shape
    btVector3 center = (minCoords + maxCoords) * 0.5f;

    // Calculate the size of the shape (length of its edge)
    float shapeSize = (maxCoords - minCoords).length();

    // Calculate the size of the tetrahedron (edge length)
    float tetrahedronSize = shapeSize * std::sqrt(6.0f);

    // Calculate the scale factor to transform the unit tetrahedron to the desired size
    float scaleFactor = tetrahedronSize / std::sqrt(1.5f);

    // Determine the vertices of the scaled unit tetrahedron
    btVector3 vertex1(center.x() - scaleFactor, center.y() - scaleFactor, center.z() - scaleFactor);
    btVector3 vertex2(center.x() + scaleFactor, center.y() - scaleFactor, center.z() - scaleFactor);
    btVector3 vertex3(center.x(), center.y() + scaleFactor, center.z() - scaleFactor);
    btVector3 vertex4(center.x(), center.y(), center.z() + scaleFactor);

    tetrahedron.color = meshColor;

    tetrahedron.facets.resize(4);
    tetrahedron.facets[0].vertices = { vertex1, vertex2, vertex3 };
    tetrahedron.facets[1].vertices = { vertex1, vertex2, vertex4 };
    tetrahedron.facets[2].vertices = { vertex1, vertex3, vertex4 };
    tetrahedron.facets[3].vertices = { vertex2, vertex3, vertex4 };
    // Populate the vertices as a single array for rendering
    tetrahedron.verticesAsSingleArr.clear();
    for (const auto& facet : tetrahedron.facets) {
        for (const auto& vertex : facet.vertices) {
            tetrahedron.verticesAsSingleArr.push_back(vertex.x());
            tetrahedron.verticesAsSingleArr.push_back(vertex.y());
            tetrahedron.verticesAsSingleArr.push_back(vertex.z());
        }
    }

    std::set<btVector3, btVector3Comparator> uniqueVertices;
    for (auto & facet : tetrahedron.facets) {
        for (auto & vertex : facet.vertices) {
            if (uniqueVertices.find(vertex) == uniqueVertices.end()) { //If not found add it
                uniqueVertices.insert(vertex);
            }
        }
    }
    
    tetrahedron.VAO = createTetrahedronVAO(tetrahedron);

    tetrahedron.allSingularVertices = std::set<btVector3, btVector3Comparator>( uniqueVertices.begin(), uniqueVertices.end());
    return tetrahedron;
}

void generateVerticesFromMesh(Mesh meshModel, std::vector<btVector3>& meshVertices) {
    std::vector<Vertex> vertices;

    for (auto& index : meshModel.indices) {
        Vertex vertex = meshModel.vertices[index];
        vertices.push_back(vertex);
    }

    std::set<btVector3> uniqueVertices;
    for (auto& vertex : vertices) {
        btVector3 convertedVertex = fromVertexToBtVector3(vertex);
        if (uniqueVertices.find(convertedVertex) == uniqueVertices.end())
            uniqueVertices.insert(convertedVertex);
    }

    meshVertices.reserve(uniqueVertices.size());

    for (auto& vertex : uniqueVertices) {
        meshVertices.push_back(vertex);
    }

}

void fillVertexData(std::vector<TriangleFacet> facets, glm::vec3 color, float vertices[]) {
    std::vector<float> verticeAndColorssAsSingleArr;

    for (auto& facet : facets) {
        for (auto& vertex : facet.vertices) {
            std::vector<float> vecComponents = generateVerticesArrayFromBtVector3(vertex);
            verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.end(), vecComponents.begin(), vecComponents.end());
        }
    }

    std::vector<float> colorAsFloatVec = { color.r, color.g, color.b };

    for (int i = 0; i < verticeAndColorssAsSingleArr.size();) {
        verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.begin() + i + 3, colorAsFloatVec.begin(), colorAsFloatVec.end());
        i += 6;
    }
    vectorToFloatArray(verticeAndColorssAsSingleArr, vertices);
}

unsigned int createTetrahedronVAO(Tetrahedron tetra) {
    unsigned int tetraVBO, tetraVAO;
    glGenVertexArrays(1, &tetraVAO);
    glGenBuffers(1, &tetraVBO);

    glBindVertexArray(tetraVAO);

    glBindBuffer(GL_ARRAY_BUFFER, tetraVBO);
    //need to pass this to OpenGL as its simpler to handle strides and stuff
    float vertices[GL_TOTAL_VERTICES_FLOAT_VALUES_PER_TETRA * VERTEX_ATTRIBUTES];

    fillVertexData(tetra.facets, tetra.color, vertices);

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * VERTEX_ATTRIBUTES) * sizeof(float), (void*)0); // (VERTICES_PER_TETRA_FACET*2) to account for vertex color
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * VERTEX_ATTRIBUTES) * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    return tetraVAO;
}

std::vector<btVector3> convertToVector(std::set<btVector3> s)
{
    std::vector<btVector3> v;
    for (btVector3 x : s) {
        v.push_back(x);
    }
    return v;
}

std::vector<float> convertVertexVectorToFlatFloatArr(std::set<Vertex> allVertices) {
    std::vector<Vertex> v(allVertices.begin(), allVertices.end());
    return convertVertexVectorToFlatFloatArr(v);
}

std::vector<float> convertVertexVectorToFlatFloatArr(std::set<btVector3, btVector3Comparator> allVertices) {
    std::set<Vertex> temp = btVectorSetToVertexSet(allVertices);
    std::vector<Vertex> v(temp.begin(), temp.end());
    return convertVertexVectorToFlatFloatArr(v);
}

std::set<Vertex> btVectorSetToVertexSet(std::set<btVector3, btVector3Comparator> allVertices) {
    std::set<Vertex> toFill;
    for (auto& el : allVertices) {
        toFill.insert(btVectorToVertex(el));
    }
    return toFill;
}

std::vector<float> convertVertexVectorToFlatFloatArr(std::vector<Vertex> allVertices) {
    std::vector<float> allVerticesAsFloatArr;
    for (auto& vertice : allVertices) {
        std::vector<float> vectorComponents = generateVerticesArrayFromVertex(vertice);
        allVerticesAsFloatArr.insert(allVerticesAsFloatArr.end(), vectorComponents.begin(), vectorComponents.end());
    }
    return allVerticesAsFloatArr;
}

Vertex btVectorToVertex(btVector3 v) {
    return { { (float)v.getX(), (float)v.getY(), (float)v.getZ() } };
}

std::vector<float> generateVerticesArrayFromBtVector3(btVector3 v) {
    return { (float)v.getX(), (float)v.getY(), (float)v.getZ() };
}

btVector3 fromVertexToBtVector3(Vertex v) {
    return btVector3(v.Position.x, v.Position.y, v.Position.z);
}

void vectorToFloatArray(const std::vector<float>& vec, float arr[]) {
    for (size_t i = 0; i < vec.size(); i++) {
        arr[i] = vec[i];
    }
}

std::vector<float> generateVerticesArrayFromVertex(Vertex v) {
    return { (float)v.Position.x, (float)v.Position.y, (float)v.Position.z };
}

bool areBtVector3Equal(btVector3 v1, btVector3 v2) {
    return v1.getX() == v2.getX() && v1.getY() == v2.getY() && v1.getZ() == v2.getZ();
}