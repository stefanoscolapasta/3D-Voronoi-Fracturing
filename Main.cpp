#define STB_IMAGE_IMPLEMENTATION
#define GL_INCLUDE_NONE
#include "incl/stb_image.h"
#include <iostream>
#include "incl/shader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include "incl/cube.h"
#include "incl/camera.h"
#include "incl/ground.h"
#include "incl/model.h"
#include "incl/physicsEngine.h"
#include "incl/Collision.h"
#include "incl/GeometricAlgorithms.h"
#include <GLFW/glfw3.h>

unsigned int generateCubeVAO(float vertices[]);
bool checkForCollisionBetweenRbsAB(PhysicsEngineAbstraction pe, btRigidBody* rigidbodyToCheckA, btRigidBody* rigidbodyToCheckB);


int main()
{
    srand(time(NULL));
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
    Shader ourShader("shaders/vshader.vert", "shaders/fshader.frag");

    // load models
    // -----------

    Model model("monkey/monkey.obj");

    PhysicsEngineAbstraction pe;
    std::vector<btVector3> generatedVerticesFromMesh;
    generateVerticesFromMesh(model.meshes[0], generatedVerticesFromMesh);
    Tetrahedron tetraEncapsulatingMesh = CreateTetrahedronAroundShape(generatedVerticesFromMesh, glm::vec3(1, 1, 1));


    VoronoiFracturing vorFrac(tetraEncapsulatingMesh, pe);
    int count = 0;
    for (int i = 0; i < 100;i++) {
        btVector3 randomPoint = extractRandomPointInsideTetrahedron(tetraEncapsulatingMesh);
        vorFrac.insertOnePoint(randomPoint, cubePositions[0]);
        count += 1;
        std::cout << count << "\n";
    }

    //model rigidbody
    std::set<btVector3, btVector3Comparator> modelVertices;
    vorFrac.uniqueVerticesFromModel(modelVertices, model);
    btRigidBody* modelRigidBody = pe.generateMeshRigidbody(cubePositions[0], modelVertices, btVector3(1.0f, 1.0f, 1.0f));
    pe.dynamicsWorld->addRigidBody(modelRigidBody, 1, 1);

    //terrain
    unsigned int cubeVAO = generateCubeVAO(cubeVertices);
    btRigidBody* cubeTerrainRigidbody = pe.generateStaticCubeRigidbody(cubePositions[1], btVector3(5.0f, 0.5f, 5.0f), btVector3(1.0f, 1.0f, 1.0f));
    pe.dynamicsWorld->addRigidBody(cubeTerrainRigidbody, 1, 1);

    MyContactResultCallback collisionResult;
    bool isCollided = false;

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


        if (isSimulationStarted()) {
            pe.dynamicsWorld->stepSimulation(getDeltaTime(), 10);
        }

        //mesh collides to the ground
        if (checkForCollisionBetweenRbsAB(pe, cubeTerrainRigidbody, modelRigidBody)) {
             if (!isCollided) {
                isCollided = true;
            }
            
             btTransform trans;
             modelRigidBody->getMotionState()->getWorldTransform(trans);
             btVector3 meshCenter = trans.getOrigin();
            pe.dynamicsWorld->removeRigidBody(modelRigidBody);
            pe.dynamicsWorld->removeCollisionObject(modelRigidBody);
            std::vector<VoronoiMesh> voronoiResult=vorFrac.convertToVoronoi( meshCenter+btVector3(0.0f,1.0f,0.0f));
        }

        if (!isCollided) {
            //MODEL DRAWING
            ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(modelRigidBody));
            model.Draw(ourShader);
        }
        else {
            for (auto& vorRigidbody : vorFrac.vorRigidBodies) {

                VoronoiMesh mesh = vorFrac.vorRigidBodyToMesh[vorRigidbody];
                createVoronoiVAO(mesh);
                glBindVertexArray(mesh.VAO);
                glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.EBO);
                ourShader.setMat4("model", pe.getUpdatedGLModelMatrix(vorRigidbody));
                // Draw the mesh using indexed rendering
                glDrawElements(GL_TRIANGLE_STRIP, mesh.nindices, GL_UNSIGNED_INT, 0);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
            }
        }
        
        

        glBindVertexArray(cubeVAO);
        glm::mat4 model = pe.getUpdatedGLModelMatrix(cubeTerrainRigidbody);
        model = glm::scale(model, glm::vec3(10.0f, 1.0f, 10.0f));
        ourShader.setMat4("model", model);
        glDrawArrays(GL_LINE_STRIP, 0, FACETS_PER_CUBE * VERTICES_PER_CUBE_FACET);

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
        //... here you can check for obA�s and obB�s user pointer again to see if the collision is alien and bullet and in that case initiate deletion.
    }
    return false;
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

