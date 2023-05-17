#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include "cube.h"
#include "shader.h"
#include "camera.h"
#include "ground.h"
#include "model.h"
#include "physicsEngine.h"
#include <iostream>
#include "Collision.h"
#include "GeometricAlgorithms.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);
unsigned int generateCubeVAO(float vertices[]);
bool checkForCollisionBetweenRbsAB(PhysicsEngineAbstraction pe, btRigidBody* rigidbodyToCheckA, btRigidBody* rigidbodyToCheckB);
    // settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 7.5f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;
bool startSimulation = false;

int main()
{
    int i, j;
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }


    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    // build and compile shaders
    // -------------------------
    Shader ourShader("vshader.vert", "fshader.frag");

    // load models
    // -----------


    Model *tetrahedronForTest = new Model("geom/tetrahedron.obj");

    PhysicsEngineAbstraction pe;
    VoronoiFracturing vorFrac(tetrahedronForTest, pe);
    //REMINDER: btCollisionObject is parentClass of btRigidBody
    //MAKE FUNCTION FOR THIS
    unsigned int cubeVAO = generateCubeVAO(cubeVertices);

    //btRigidBody* cubeRigidBody = pe.generateCubeRigidbody(cubePositions[0], btVector3(0.5f, 0.5f, 0.5f), btVector3(1.0f, 1.0f, 1.0f));
    //btRigidBody* groundRigidBody = pe.generateGroundRigidbody(groundPositions[0]);
    btRigidBody* cubeTerrainRigidbody = pe.generateStaticCubeRigidbody(cubePositions[1], btVector3(5.0f, 0.5f, 5.0f), btVector3(1.0f, 1.0f, 1.0f));
    // add the ground rigid body to the physics world
    pe.dynamicsWorld->addRigidBody(cubeTerrainRigidbody, 1, 1);
    //pe.dynamicsWorld->addRigidBody(groundRigidBody,1,1);
    btRigidBody* initialTetra = *(vorFrac.tetraRigidbodies.begin());
    //I added the centroid (kinda)
    //Callback to use when checking collisions
    MyContactResultCallback collisionResult;

    while (!glfwWindowShouldClose(window))
    {
        
        //Calculate deltatime
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------illy
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // also clear the depth buffer now!

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("view", view);
        
        // UPDATE SIMULATION
        if (startSimulation) {
            pe.dynamicsWorld->stepSimulation(deltaTime, 10);
        }
             
        //Here I check for  collision, if collision happened I generate a point linearly interpolating the contact point and the centroid
        //This point will be used to generate new tetras and give effect of breaking
        //pe.dynamicsWorld->contactPairTest()
        if (checkForCollisionBetweenRbsAB(pe, cubeTerrainRigidbody, initialTetra)) { 
            vorFrac.insertOnePoint(btVector3(0.0f, 0.0f, 0.0f), initialTetra, initialTetra->getCenterOfMassTransform().getOrigin()); //*(vorFrac.tetraRigidbodies.begin()) is used to get the """first""" element in the set (sets are not strictly ordered)
        }
        //ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(cubeRigidBody));
        for (auto& tetraRigidbody : vorFrac.tetraRigidbodies) {

            glBindVertexArray(vorFrac.tetraToVAO[tetraRigidbody]);
            ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(tetraRigidbody));
            //Here we need the VAO for each tetrahedron as their shape is not always the same
            // 
            glDrawArrays(GL_TRIANGLES, 0, 36);
            //tetrahedronForTest->Draw(ourShader);
        }
        glBindVertexArray(cubeVAO);
        glm::mat4 model = pe.getUpdatedGLModelMatrix(cubeTerrainRigidbody);
        model = glm::scale(model, glm::vec3(10.0f, 1.0f, 10.0f));
        ourShader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &cubeVAO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

bool checkForCollisionBetweenRbsAB(PhysicsEngineAbstraction pe, btRigidBody* rigidbodyToCheckA, btRigidBody* rigidbodyToCheckB) {
    int numManifolds = pe.dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = pe.dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        const btRigidBody* rbA = btRigidBody::upcast(obA);
        const btRigidBody* rbB = btRigidBody::upcast(obB);
        if ((rbA == rigidbodyToCheckA && rbB == rigidbodyToCheckB) || (rbB == rigidbodyToCheckA && rbA == rigidbodyToCheckB)) {
            //contactManifold->getBody0()->getWorldTransform().getOrigin();
            return true;
        }
        //... here you can check for obA´s and obB´s user pointer again to see if the collision is alien and bullet and in that case initiate deletion.
    }
    return false;
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
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
        startSimulation = true;
}

unsigned int generateCubeVAO(float vertices[]) {
    unsigned int cubeVBO, cubeVAO;
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);

    glBindVertexArray(cubeVAO);

    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0); 
    // color attribute
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    
    return cubeVAO;
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

