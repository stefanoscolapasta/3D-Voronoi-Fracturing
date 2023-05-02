#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "camera.h"

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//Camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

int main()
{
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

    // build and compile our shader zprogram
    // ------------------------------------
    Shader ourShader("texture.vs", "texture.fs");

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        //Vertex coords       //Colors
        -0.5f, -0.5f, -0.5f,  0.0f,  0.3f, 0.3f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.3f, 0.3f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.3f, 0.3f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.3f, 0.3f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.3f, 0.3f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.3f, 0.3f,

        -0.5f, -0.5f,  0.5f,  0.0f, 0.4f, 0.2f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.4f, 0.2f,
         0.5f,  0.5f,  0.5f,  0.0f, 0.4f, 0.2f,
         0.5f,  0.5f,  0.5f,  0.0f, 0.4f, 0.2f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.4f, 0.2f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.4f, 0.2f,

        -0.5f,  0.5f,  0.5f,  0.0f, 0.5f, 0.1f,
        -0.5f,  0.5f, -0.5f,  0.0f, 0.5f, 0.1f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.5f, 0.1f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.5f, 0.1f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.5f, 0.1f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.5f, 0.1f,

         0.5f,  0.5f,  0.5f,  0.0f, 0.6f, 0.0f,
         0.5f,  0.5f, -0.5f,  0.0f, 0.6f, 0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 0.6f, 0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 0.6f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.6f, 0.0f,
         0.5f,  0.5f,  0.5f,  0.0f, 0.6f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, 0.7f, 0.4f,
         0.5f, -0.5f, -0.5f,  0.0f, 0.7f, 0.4f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.7f, 0.4f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.7f, 0.4f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.7f, 0.4f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.7f, 0.4f,

        -0.5f,  0.5f, -0.5f,  0.0f, 0.8f, 0.3f,
         0.5f,  0.5f, -0.5f,  0.0f, 0.8f, 0.3f,
         0.5f,  0.5f,  0.5f,  0.0f, 0.8f, 0.3f,
         0.5f,  0.5f,  0.5f,  0.0f, 0.8f, 0.3f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.8f, 0.3f,
        -0.5f,  0.5f, -0.5f,  0.0f, 0.8f, 0.3f,
    };
    // world space positions of our cubes
    glm::vec3 cubePositions[] = {
        glm::vec3(0.0f,  0.0f,  0.0f),
        glm::vec3(2.0f,  5.0f, -15.0f),
        glm::vec3(-1.5f, -2.2f, -2.5f),
        glm::vec3(-3.8f, -2.0f, -12.3f),
        glm::vec3(2.4f, -0.4f, -3.5f),
        glm::vec3(-1.7f,  3.0f, -7.5f),
        glm::vec3(1.3f, -2.0f, -2.5f),
        glm::vec3(1.5f,  2.0f, -2.5f),
        glm::vec3(1.5f,  0.2f, -1.5f),
        glm::vec3(-1.3f,  1.0f, -1.5f)
    };
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    
    // tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
    // -------------------------------------------------------------------------------------------
    ourShader.use();
    // render loop
    // -----------
    int i = 0;

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
        // ------
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

        // render boxes
        glBindVertexArray(VAO);

        // calculate the model matrix for each object and pass it to shader before drawing
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, cubePositions[0]);

        float angle = 0; // *i

        model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));

        ourShader.setMat4("model", model);

        glDrawArrays(GL_TRIANGLES, 0, 36);
        

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

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
