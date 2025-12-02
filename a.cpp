#include <cstdio>
#include <cmath>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <random>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define nBALLS 500
#define QUALITY 10 // vertex count (circle points)
#define deltaTime 1.5e-2

#define RANDCONST 10
#define MAXRADIUS 4
#define G -9.8f
#define maxABSAcceleration 10.0f

// Vertex shader
const char* vertexShaderSrc = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec3 aColor;

out vec3 vColor;

void main() {
    float x = (aPos.x / 400) - 1.0;
    float y = 1.0 - (aPos.y / 300);
    gl_Position = vec4(x, y, 0.0, 1.0);
    vColor = aColor;
}
)";

// Fragment shader
const char* fragmentShaderSrc = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";


void precomputeCircle(float shapeVerts[], int quality) {
    float angleStep = 2.0f * M_PI / quality;
    for (int i = 0; i < quality; ++i) {
        shapeVerts[i * 2 + 0] = cosf(i * angleStep);
        shapeVerts[i * 2 + 1] = sinf(i * angleStep);
    }
}

void updateBallVertices(float* outVerts, float cx, float cy, float radius, float* shapeVerts, int quality) {
    outVerts[0] = cx;
    outVerts[1] = cy;
    for (int i = 0; i < quality; ++i) {
        outVerts[(i + 1) * 2 + 0] = cx + radius * shapeVerts[i * 2];
        outVerts[(i + 1) * 2 + 1] = cy + radius * shapeVerts[i * 2 + 1];
    }
    // Close fan by repeating first perimeter vertex
    outVerts[(quality + 1) * 2 + 0] = outVerts[2];
    outVerts[(quality + 1) * 2 + 1] = outVerts[3];
}

void handleBoundaryCollision(float* prevPos, float* pos, int index, int radius) {
    // Left boundary
    if (pos[index] < radius) {
        pos[index] = (float)radius;
        prevPos[index] = pos[index] + (pos[index] - prevPos[index]); // reflect velocity x
    }
    // Right boundary
    else if (pos[index] > WINDOW_WIDTH - radius) {
        pos[index] = WINDOW_WIDTH - radius;
        prevPos[index] = pos[index] + (pos[index] - prevPos[index]); // reflect velocity x
    }

    // Top boundary
    if (pos[index + 1] < radius) {
        pos[index + 1] = (float)radius;
        prevPos[index + 1] = pos[index + 1] + (pos[index + 1] - prevPos[index + 1]); // reflect velocity y
    }
    // Bottom boundary
    else if (pos[index + 1] > WINDOW_HEIGHT - radius) {
        pos[index + 1] = WINDOW_HEIGHT - radius;
        prevPos[index + 1] = pos[index + 1] + (pos[index + 1] - prevPos[index + 1]); // reflect velocity y
    }
}

void verlet(float* prevPos, float* pos, float* acc, int* radius, int nBalls, float dt = 1.0f) {
    for (int i = 0; i < nBalls; i++) {
        int index = i * 2;

        float nextX = pos[index] + (pos[index] - prevPos[index]) + acc[index] * dt * dt;
        float nextY = pos[index + 1] + (pos[index + 1] - prevPos[index + 1]) + acc[index + 1] * dt * dt;

        prevPos[index] = pos[index];
        prevPos[index + 1] = pos[index + 1];

        pos[index] = nextX;
        pos[index + 1] = nextY;

        handleBoundaryCollision(prevPos, pos, index, radius[i]);
    }
}





void initializeBalls(int* ballRadius, float* ballPrevPositions, float* ballPositions,
                     float* ballAcceleration, float* ballColors, int nBalls) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::uniform_real_distribution<float> distPosX(0.0f, 1.0f);
    std::uniform_real_distribution<float> distPosY(0.0f, 1.0f);
    std::uniform_real_distribution<float> distOffset(-RANDCONST, RANDCONST);
    std::uniform_real_distribution<float> distAccX(-maxABSAcceleration, maxABSAcceleration);
    std::uniform_int_distribution<int> distRadius(1, MAXRADIUS);
    std::uniform_real_distribution<float> distRGB(0.0f, 1.0f); // for colors

    for (int i = 0; i < nBalls; i++) {
        int radius = distRadius(gen);
        ballRadius[i] = radius;

        float marginX = static_cast<float>(radius);
        float marginY = static_cast<float>(radius);

        float posX = marginX + distPosX(gen) * (WINDOW_WIDTH - 2 * marginX);
        float posY = marginY + distPosY(gen) * (WINDOW_HEIGHT - 2 * marginY);

        ballPositions[i * 2 + 0] = posX;
        ballPositions[i * 2 + 1] = posY;

        float offsetX = distOffset(gen);
        float offsetY = distOffset(gen);

        ballPrevPositions[i * 2 + 0] = posX - offsetX;
        ballPrevPositions[i * 2 + 1] = posY - offsetY;

        float accX = distAccX(gen);
        float accY = -G;

        ballAcceleration[i * 2 + 0] = accX;
        ballAcceleration[i * 2 + 1] = accY;

        // Assign a random RGB color for each vertex in this ball
        float r = distRGB(gen);
        float g = distRGB(gen);
        float b = distRGB(gen);

        for (int j = 0; j < QUALITY + 2; ++j) {
            int idx = (i * (QUALITY + 2) + j) * 3;
            ballColors[idx + 0] = r;
            ballColors[idx + 1] = g;
            ballColors[idx + 2] = b;
        }
    }
}


// Utility: compile shader and check errors
GLuint compileShader(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (!status) {
        char buf[512];
        glGetShaderInfoLog(shader, 512, nullptr, buf);
        printf("Shader compile error: %s\n", buf);
    }
    return shader;
}

int main() {
    if (!glfwInit()) {
        printf("Failed to initialize GLFW3\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Balls Verlet Simulation", nullptr, nullptr);
    if (!window) {
        printf("Failed to create GLFW window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        printf("Failed to initialize GLEW\n");
        return 1;
    }

    // Setup shader program
    GLuint vertShader = compileShader(GL_VERTEX_SHADER, vertexShaderSrc);
    GLuint fragShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSrc);
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertShader);
    glAttachShader(shaderProgram, fragShader);
    glLinkProgram(shaderProgram);

    GLint linkStatus;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linkStatus);
    if (!linkStatus) {
        char buf[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, buf);
        printf("Shader link error: %s\n", buf);
    }

    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    // Prepare buffers
    GLuint vao, vboPositions, vboColors;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vboPositions);
    glGenBuffers(1, &vboColors);

    glBindVertexArray(vao);

    // Position attribute (location = 0)
    glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Color attribute (location = 1)
    glBindBuffer(GL_ARRAY_BUFFER, vboColors);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindVertexArray(0);

    // Initialize ball data
    std::vector<int> ballRadius(nBALLS);
    std::vector<float> ballPrevPositions(nBALLS * 2);
    std::vector<float> ballPositions(nBALLS * 2);
    std::vector<float> ballAcceleration(nBALLS * 2);
    std::vector<float> vertices(nBALLS * (QUALITY + 2) * 2);
    std::vector<float> ballColors(nBALLS * (QUALITY + 2) * 3);  // RGB per vertex

    initializeBalls(ballRadius.data(), ballPrevPositions.data(), ballPositions.data(),
                ballAcceleration.data(), ballColors.data(), nBALLS);



    // Prepare vertices storage
    float shapeVertices[(QUALITY + 2) * 2];
    precomputeCircle(shapeVertices, QUALITY);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        verlet(ballPrevPositions.data(), ballPositions.data(), ballAcceleration.data(), ballRadius.data(), nBALLS, deltaTime);

        for (int i = 0; i < nBALLS; ++i) {
            updateBallVertices(
                &vertices[i * (QUALITY + 2) * 2],
                ballPositions[i * 2], ballPositions[i * 2 + 1],
                (float)ballRadius[i],
                shapeVertices, QUALITY);
        }

        glBindVertexArray(vao);

        // Upload updated positions
        glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

        // Upload colors (static, but you can upload here once if no changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboColors);
        glBufferData(GL_ARRAY_BUFFER, ballColors.size() * sizeof(float), ballColors.data(), GL_STATIC_DRAW);

        glUseProgram(shaderProgram);

        for (int i = 0; i < nBALLS; ++i) {
            glDrawArrays(GL_TRIANGLE_FAN, i * (QUALITY + 2), QUALITY + 2);
        }


        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
