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
#include "GeometricAlgorithms.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 15.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

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

    const int numTetrahedrons = 6;
    Model *tetraModels[numTetrahedrons];
    for (i = 0; i < numTetrahedrons; i++) {
        string path = "tetra/tetra" + std::to_string(i) +".obj";
        tetraModels[i] = new Model(path);
    }
    
    
    Model *tetrahedronForTest = new Model("geom/tetrahedron");
    VoronoiFracturing vorFrac(tetrahedronForTest);

    unsigned int groundVBO, groundVAO;
    glGenVertexArrays(1, &groundVAO);
    glGenBuffers(1, &groundVBO);

    glBindVertexArray(groundVAO);

    glBindBuffer(GL_ARRAY_BUFFER, groundVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(groundVertices), groundVertices, GL_STATIC_DRAW);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(3);


    PhysicsEngineAbstraction pe;

    //btRigidBody* cubeRigidBody = pe.generateCubeRigidbody(cubePositions[0], btVector3(0.5f, 0.5f, 0.5f), btVector3(1.0f, 1.0f, 1.0f));
    btRigidBody* groundRigidBody = pe.generateGroundRigidbody(groundPositions[0]);
    // add the ground rigid body to the physics world
    //pe.dynamicsWorld->addRigidBody(cubeRigidBody);
    pe.dynamicsWorld->addRigidBody(groundRigidBody);


    // Generate tetrahedrons ...

    btRigidBody* tetrahedronRigidBodies[numTetrahedrons];
    btVector3 tetrahedronVertices[numTetrahedrons][5];

    for (i = 0; i < numTetrahedrons; i++) {
       
        int index = 0;
        for (int j = 0; j < tetraModels[i]->meshes[0].vertices.size(); j++) {
            btVector3 newPoint = btVector3(tetraModels[i]->meshes[0].vertices[j].Position.x,
                tetraModels[i]->meshes[0].vertices[j].Position.y,
                tetraModels[i]->meshes[0].vertices[j].Position.z);
            bool already_exists = false;
            
            for (int k = 0; k < j; k++) {
                if (newPoint == tetrahedronVertices[i][k]) {
                    already_exists = true;
                    break;
                }
            }
            if (!already_exists) {
                tetrahedronVertices[i][index] = newPoint;
                index++;
            }
        }
       btRigidBody* tetrahedronRigidBody = pe.generateTetrahedronRigidbody(
            cubePositions[0], // Use cube position as starting position
           tetrahedronVertices[i],
           btVector3(1.0f,1.0f,1.0f)
        );
        tetrahedronRigidBodies[i] = tetrahedronRigidBody;
        pe.dynamicsWorld->addRigidBody(tetrahedronRigidBodies[i]);
    }


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
        pe.dynamicsWorld->stepSimulation(deltaTime, 10);

        //ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(cubeRigidBody));
        for (i = 0; i < numTetrahedrons; i++){
            ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(tetrahedronRigidBodies[i]));
            tetraModels[i]->Draw(ourShader);
        }
        
        glBindVertexArray(groundVAO);
        ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(groundRigidBody));
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &groundVAO);
    glDeleteBuffers(1, &groundVBO);


    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
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

