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
#include <iostream>
#include "cube.h"
#include "shader.h"
#include "camera.h"
#include "ground.h"
#include "model.h"
#include "physicsEngine.h"
#include "GeometricAlgorithms.h"
#include "Collision.h"


unsigned int generateCubeVAO(float vertices[]);



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


    Model* tetrahedronForTest = new Model("geom/tetrahedron.obj");

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

    //I added the centroid (kinda)
    vorFrac.insertOnePoint(btVector3(0.0f, 0.0f, 0.0f), *(vorFrac.tetraRigidbodies.begin())); //*(vorFrac.tetraRigidbodies.begin()) is used to get the """first""" element in the set (sets are not strictly ordered)
    //Callback to use when checking collisions
    MyContactResultCallback collisionResult;

    while (!glfwWindowShouldClose(window))
    {

        //Calculate deltatime
        float currentFrame = glfwGetTime();
        setFrame(currentFrame - getLastFrame(), currentFrame);

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
        glm::mat4 projection = glm::perspective(glm::radians(getCamera().Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = getCamera().GetViewMatrix();
        ourShader.setMat4("view", view);

        // UPDATE SIMULATION

        pe.dynamicsWorld->stepSimulation(getDeltaTime(), 10);

        //Here I check for  collision, if collision happened I generate a point linearly interpolating the contact point and the centroid
        //This point will be used to generate new tetras and give effect of breaking
        //pe.dynamicsWorld->contactPairTest()

        //ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(cubeRigidBody));
        for (auto& tetraRigidbody : vorFrac.tetraRigidbodies) {

            pe.dynamicsWorld->contactPairTest(tetraRigidbody, cubeTerrainRigidbody, collisionResult); //Check collision with ground use contactTest to check will all rigidbodies

            if (collisionResult.m_closestDistanceThreshold > 0) {
                std::cout << "Collision with ground"; //Does not work ffs
            }

            glBindVertexArray(vorFrac.tetraToVAO[tetraRigidbody]);
            ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(tetraRigidbody));
            //Here we need the VAO for each tetrahedron as their shape is not always the same
            // 
            glDrawArrays(GL_LINE_STRIP, 0, 36);
            //tetrahedronForTest->Draw(ourShader);
        }
        glBindVertexArray(cubeVAO);
        glm::mat4 model = pe.getUpdatedGLModelMatrix(cubeTerrainRigidbody);
        model = glm::scale(model, glm::vec3(10.0f, 1.0f, 10.0f));
        ourShader.setMat4("model", model);
        glDrawArrays(GL_LINE_STRIP, 0, 36);

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



unsigned int generateCubeVAO(float vertices[]) {
    unsigned int cubeVBO, cubeVAO;
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);

    glBindVertexArray(cubeVAO);

    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    return cubeVAO;
}

