// ModernOpenGL_Main.cpp
// Build: link with glfw, glew, imgui + imgui_impl_glfw, imgui_impl_opengl3, glm
// Requires your existing Line.h (with Node/Line definitions).

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <cstring>
#include <limits>
#include <string>
#include <random>


#include "Line.h"

#define WIDTH 800
#define HEIGHT 600
#define BALL_QUALITY 20
#define BALL_RADIUS 10.0f
#define DT 0.1f
#define GRAVITY -10.0f
#define DAMPING 0.999f
#define PICK_RADIUS 15.0f

// Globals
GLuint circleVBO = 0, circleVAO = 0;
GLuint lineVAO = 0, lineVBO = 0; // reused dynamic buffer for lines
GLuint shaderProgram = 0;

GLFWwindow* windowPtr = nullptr;

int fbWidth = WIDTH;
int fbHeight = HEIGHT;
int winWidth = WIDTH;
int winHeight = HEIGHT;

glm::mat4 gProjection(1.0f);

std::vector<Line*> lines;
bool paused = false;

enum OPTIONS {
    DRAGGING,
    INSERTING,
    DELETING,
    CUTTING,
    TOGGLING,
};
OPTIONS m_Mode = OPTIONS::TOGGLING;

bool isDragging = false;
glm::vec2 dragStart(0.0f);
glm::vec2 dragEnd(0.0f);
Line* dragLine = nullptr;  // line on which drag started
Node* dragNodeA = nullptr;
Node* dragNodeB = nullptr;
float gInsertDelta = 20.0f;   // default spacing between nodes
int   gInsertCount = 10;      // number of nodes to insert

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis(0, gInsertCount);
// ---------------------------
// Shader helpers
// ---------------------------
static const char* kVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPos;

uniform mat4 uProjection;
uniform mat4 uModel;

void main() {
    gl_Position = uProjection * uModel * vec4(aPos, 0.0, 1.0);
}
)GLSL";

static const char* kFragmentShader = R"GLSL(
#version 330 core
out vec4 FragColor;
uniform vec3 uColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
)GLSL";

GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0; glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetShaderInfoLog(s, len, nullptr, log.data());
        std::cerr << "Shader compile error: " << log << std::endl;
    }
    return s;
}

GLuint createProgram(const char* vs, const char* fs) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0; glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(p, len, nullptr, log.data());
        std::cerr << "Program link error: " << log << std::endl;
    }
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

// ---------------------------
// Math & picking helpers
// ---------------------------
glm::vec2 screenToWorld(GLFWwindow* window, double xpos, double ypos) {
    int wW, wH;
    glfwGetWindowSize(window, &wW, &wH);
    int fW, fH;
    glfwGetFramebufferSize(window, &fW, &fH);

    float scaleX = (wW > 0) ? (float)fW / (float)wW : 1.0f;
    float scaleY = (wH > 0) ? (float)fH / (float)wH : 1.0f;

    float fbX = (float)xpos * scaleX;
    float fbY = (float)ypos * scaleY;

    return glm::vec2(fbX, (float)(fH) - fbY);
}

float distSquared(const float* pos, const glm::vec2& point2D) {
    float dx = pos[0] - point2D.x;
    float dy = pos[1] - point2D.y;
    return dx*dx + dy*dy;
}

Node* findClosestNode(Line& line, const glm::vec2& clickPos, float maxDist) {
    Node* curr = line.root;
    Node* closest = nullptr;
    float maxDistSq = maxDist * maxDist;
    float bestDistSq = maxDistSq;

    while (curr) {
        float dSq = distSquared(curr->position, clickPos);
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            closest = curr;
        }
        curr = curr->getNext();
    }
    return closest;
}

float pointSegmentDistSq(const glm::vec2& p, const glm::vec2& v, const glm::vec2& w) {
    float l2 = glm::length2(w - v);
    if (l2 == 0.0f) return glm::length2(p - v);
    float t = glm::dot(p - v, w - v) / l2;
    t = glm::clamp(t, 0.0f, 1.0f);
    glm::vec2 projection = v + t * (w - v);
    return glm::length2(p - projection);
}

struct LineSegmentHit {
    Line* line = nullptr;
    Node* nodeA = nullptr;
    Node* nodeB = nullptr;
    float distSq = std::numeric_limits<float>::max();
};

LineSegmentHit findClosestSegmentInAllLines(const glm::vec2& clickPos, float maxDist) {
    LineSegmentHit hit;
    float maxDistSq = maxDist * maxDist;

    for (Line* line : lines) {
        Node* curr = line->root;
        while (curr && curr->getNext()) {
            glm::vec2 v(curr->position[0], curr->position[1]);
            glm::vec2 w(curr->getNext()->position[0], curr->getNext()->position[1]);
            float distSq = pointSegmentDistSq(clickPos, v, w);
            if (distSq < maxDistSq && distSq < hit.distSq) {
                hit.line = line;
                hit.nodeA = curr;
                hit.nodeB = curr->getNext();
                hit.distSq = distSq;
            }
            curr = curr->getNext();
        }
    }
    return hit;
}

// ---------------------------
// Physics helpers (unchanged logic)
// ---------------------------
void enforceMaxDistance(Node* a, Node* b, float delta) {
    float dir[3];
    float distSq = 0.0f;

    for (int i = 0; i < 3; ++i) {
        dir[i] = b->position[i] - a->position[i];
        distSq += dir[i] * dir[i];
    }

    float dist = sqrtf(distSq);
    if (dist < 1e-6f) return;

    float diff = (dist - delta) / dist;
    float offset[3] = {
        dir[0] * 0.5f * diff,
        dir[1] * 0.5f * diff,
        dir[2] * 0.5f * diff
    };

    if (!a->fixed && !b->fixed) {
        for (int i = 0; i < 3; ++i) {
            a->position[i] += offset[i];
            b->position[i] -= offset[i];
        }
    } else if (!a->fixed) {
        for (int i = 0; i < 3; ++i)
            a->position[i] += offset[i] * 2.0f;
    } else if (!b->fixed) {
        for (int i = 0; i < 3; ++i)
            b->position[i] -= offset[i] * 2.0f;
    }
}

void resolveNodeCollision(Node* a, Node* b, float radiusSum) {
    float dir[3];
    float distSq = 0.0f;

    for (int i = 0; i < 3; ++i) {
        dir[i] = b->position[i] - a->position[i];
        distSq += dir[i] * dir[i];
    }

    float minDist = radiusSum;
    float minDistSq = minDist * minDist;

    if (distSq >= minDistSq || distSq < 1e-6f) return;

    float dist = sqrtf(distSq);
    float overlap = minDist - dist;
    float offsetAmount = overlap / dist * 0.5f;

    float offset[3];
    for (int i = 0; i < 3; ++i)
        offset[i] = dir[i] * offsetAmount;

    if (!a->fixed && !b->fixed) {
        for (int i = 0; i < 3; ++i) {
            a->position[i] -= offset[i];
            b->position[i] += offset[i];
        }
    } else if (!a->fixed) {
        for (int i = 0; i < 3; ++i)
            a->position[i] -= offset[i] * 2.0f;
    } else if (!b->fixed) {
        for (int i = 0; i < 3; ++i)
            b->position[i] += offset[i] * 2.0f;
    }
}

void enforceWallCollision(Node* node, float radius) {
    if (node->fixed) return;

    if (node->position[0] < radius)
        node->position[0] = radius;
    else if (node->position[0] > fbWidth - radius)
        node->position[0] = fbWidth - radius;

    if (node->position[1] < radius)
        node->position[1] = radius;
    else if (node->position[1] > fbHeight - radius)
        node->position[1] = fbHeight - radius;
}

void applyGravity(Line& line, float gravity = GRAVITY, float timeStep = DT) {
    std::vector<Node*> nodeList;
    Node* curr = line.root;
    while (curr) {
        nodeList.push_back(curr);
        curr = curr->getNext();
    }

    for (Node* node : nodeList) {
        if (node->fixed) continue;
        if (node == dragNodeA) {
            double xpos, ypos;
            glfwGetCursorPos(windowPtr, &xpos, &ypos);
            glm::vec2 p = screenToWorld(windowPtr, xpos, ypos);
            node->position[0] = p[0];
            node->position[1] = p[1];
            continue;
        };

        float* pos = node->position;
        float* prev = node->previousPos;
        float temp[3] = { pos[0], pos[1], pos[2] };

        pos[0] += (pos[0] - prev[0]) * DAMPING;
        pos[1] += (pos[1] - prev[1]) * DAMPING + gravity * timeStep * timeStep;
        pos[2] += (pos[2] - prev[2]) * DAMPING;

        memcpy(prev, temp, sizeof(float) * 3);
    }

    const int iterations = 8;
    for (int it = 0; it < iterations; ++it) {
        for (size_t i = 0; i + 1 < nodeList.size(); ++i) {
            enforceMaxDistance(nodeList[i], nodeList[i + 1], line.delta);
        }
    }
}

// ---------------------------
// Rendering (modern GL)
// ---------------------------
void updateProjection() {
    gProjection = glm::ortho(0.0f, (float)fbWidth, 0.0f, (float)fbHeight, -1.0f, 1.0f);
    glUseProgram(shaderProgram);
    GLint locProj = glGetUniformLocation(shaderProgram, "uProjection");
    glUniformMatrix4fv(locProj, 1, GL_FALSE, glm::value_ptr(gProjection));
    glUseProgram(0);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    fbWidth = width;
    fbHeight = height;
    glViewport(0, 0, fbWidth, fbHeight);
    updateProjection();
}

// Static unit circle VAO (triangle fan)
void createUnitCircle() {
    std::vector<float> circleVertices;
    circleVertices.reserve((BALL_QUALITY + 2) * 2);
    circleVertices.push_back(0.0f);
    circleVertices.push_back(0.0f);
    for (int i = 0; i <= BALL_QUALITY; ++i) {
        float angle = 2.0f * (float)M_PI * i / BALL_QUALITY;
        circleVertices.push_back(cosf(angle));
        circleVertices.push_back(sinf(angle));
    }

    glGenVertexArrays(1, &circleVAO);
    glGenBuffers(1, &circleVBO);
    glBindVertexArray(circleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, circleVBO);
    glBufferData(GL_ARRAY_BUFFER, circleVertices.size() * sizeof(float), circleVertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindVertexArray(0);
}

// Reusable dynamic line VAO/VBO
void createLineBuffer() {
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
    glBindVertexArray(0);
}

void drawBall(const glm::vec2& pos, const glm::vec3& color) {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(pos, 0.0f));
    model = glm::scale(model, glm::vec3(BALL_RADIUS, BALL_RADIUS, 1.0f));

    glUseProgram(shaderProgram);
    GLint locModel = glGetUniformLocation(shaderProgram, "uModel");
    GLint locColor = glGetUniformLocation(shaderProgram, "uColor");
    glUniformMatrix4fv(locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(locColor, 1, glm::value_ptr(color));

    glBindVertexArray(circleVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, BALL_QUALITY + 2);
    glBindVertexArray(0);
}

void renderLine(Line& line) {
    // Gather vertices
    std::vector<glm::vec2> vertices;
    Node* curr = line.root;
    while (curr) {
        vertices.emplace_back(curr->position[0], curr->position[1]);
        curr = curr->getNext();
    }
    if (vertices.empty()) return;

    // Upload to dynamic buffer
    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);

    // Render line strip
    glUseProgram(shaderProgram);
    glm::mat4 model(1.0f);
    GLint locModel = glGetUniformLocation(shaderProgram, "uModel");
    GLint locColor = glGetUniformLocation(shaderProgram, "uColor");
    glUniformMatrix4fv(locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(locColor, 0.0f, 1.0f, 0.0f);

    glDrawArrays(GL_LINE_STRIP, 0, (GLsizei)vertices.size());

    glBindVertexArray(0);

    // Draw balls for nodes
    curr = line.root;
    while (curr) {
        glm::vec3 color = curr->fixed ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        drawBall(glm::vec2(curr->position[0], curr->position[1]), color);
        curr = curr->getNext();
    }
}

// Render drag line (preview)
void renderDragLine() {
    if (!isDragging) return;
    if (!dragNodeA->fixed) return;
    glm::vec2 verts[2] = { dragStart, dragEnd };

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);

    glUseProgram(shaderProgram);
    glm::mat4 model(1.0f);
    GLint locModel = glGetUniformLocation(shaderProgram, "uModel");
    GLint locColor = glGetUniformLocation(shaderProgram, "uColor");
    glUniformMatrix4fv(locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(locColor, 1.0f, 1.0f, 0.0f);

    glBindVertexArray(lineVAO);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

// ---------------------------
// Line creation
// ---------------------------
void createNewLine(const glm::vec2& start, const glm::vec2& end, int numPoints) {
    if (numPoints < 2) {
        std::cout << "Number of nodes must be >= 2.\n";
        return;
    }
    Line* newLine = new Line();

    float dir[3] = { (end.x - start.x) / (numPoints - 1), (end.y - start.y) / (numPoints - 1), 0.f };
    float pos[3] = { start.x, start.y, 0.f };

    newLine->root = new Node(pos, nullptr);
    Node* prev = newLine->root;

    for (int i = 1; i < numPoints; ++i) {
        pos[0] = start.x + dir[0] * i;
        pos[1] = start.y + dir[1] * i;
        pos[2] = 0.f;
        Node* nextNode = new Node(pos, prev);
        prev->setNext(nextNode);
        prev = nextNode;
    }
    newLine->end = prev;

    float delta = glm::distance(
        glm::vec2(newLine->root->position[0], newLine->root->position[1]),
        glm::vec2(newLine->root->getNext()->position[0], newLine->root->getNext()->position[1])
    );
    newLine->delta = delta;

    lines.push_back(newLine);
    std::cout << "Created new line with " << numPoints << " nodes.\n";
}

// ---------------------------
// Input callbacks
// ---------------------------
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        paused = !paused;
        std::cout << (paused ? "Paused" : "Unpaused") << " simulation.\n";
    }

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

    if (key == GLFW_KEY_I && action == GLFW_PRESS) {
        m_Mode = OPTIONS::INSERTING;
    }
    if (key == GLFW_KEY_T && action == GLFW_PRESS) {
        m_Mode = OPTIONS::TOGGLING;
    }
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        m_Mode = OPTIONS::DELETING;
    }
    if (key == GLFW_KEY_U && action == GLFW_PRESS) {
        m_Mode = OPTIONS::CUTTING;
    }
    if (key == GLFW_KEY_D && action == GLFW_PRESS) {
        m_Mode = OPTIONS::DRAGGING;
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    glm::vec2 clickPos = screenToWorld(window, xpos, ypos);

    if (action == GLFW_PRESS) {
        ImGuiIO& io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        if (m_Mode == OPTIONS::TOGGLING) {
            for (Line* line : lines) {
                Node* clickedNode = findClosestNode(*line, clickPos, PICK_RADIUS);
                if (clickedNode) {
                    float dSq = distSquared(clickedNode->position, clickPos);
                    if (dSq <= PICK_RADIUS * PICK_RADIUS) {
                        clickedNode->setFixed(!clickedNode->fixed);
                        std::cout << "Toggled node fixed state to " << clickedNode->fixed << std::endl;
                        return;
                    }
                }
            }
        } else if (m_Mode == OPTIONS::DRAGGING) {
            for (Line* line : lines) {
                Node* clickedNode = findClosestNode(*line, clickPos, PICK_RADIUS);
                if (clickedNode) {
                    isDragging = true;
                    dragNodeA = clickedNode;
                    dragLine = line;
                    dragStart = clickPos;
                    dragEnd = clickPos;
                    std::cout << "Started dragging from node.\n";
                    return;
                }
            }

            auto hit = findClosestSegmentInAllLines(clickPos, PICK_RADIUS);
            if (hit.line && hit.nodeA && hit.nodeB) {
                isDragging = true;
                dragStart = clickPos;
                dragEnd = clickPos;
                dragLine = hit.line;
                dragNodeA = hit.nodeA;
                dragNodeB = hit.nodeB;
                std::cout << "Started dragging to move line.\n";
            }
        } else if (m_Mode == OPTIONS::CUTTING) {
            auto hit = findClosestSegmentInAllLines(clickPos, PICK_RADIUS);
            if (hit.line && hit.nodeA && hit.nodeB) {
                hit.nodeA->next = nullptr;
                Line* newLine = new Line(hit.line->delta, 0, hit.nodeB->position);
                lines.push_back(newLine);
                newLine->root = hit.nodeB;
                newLine->root->setFixed(true);
            }
        } else if (m_Mode == OPTIONS::INSERTING) {
            float* starting = new float[3]{ clickPos.x, clickPos.y, 0.0f };

            int random = dis(gen);
            Line* newLine = new Line(gInsertDelta, gInsertCount, starting);
            lines.push_back(newLine);
            Node* root = newLine->root;
            int i = 0;
            while (i != random) {
                root = root->next;
                i++;
            };
            root->setFixed(true);
        }
        else if (m_Mode == OPTIONS::DELETING) {
            auto hit = findClosestSegmentInAllLines(clickPos, PICK_RADIUS);
            auto it = std::find(lines.begin(), lines.end(), hit.line);
            if (it != lines.end()) {
                delete *it;  // Free the memory if you allocated it with 'new'
                lines.erase(it);  // Remove from vector
            }
        }

    } else if (action == GLFW_RELEASE) {
        if (isDragging && m_Mode == OPTIONS::DRAGGING) {
            isDragging = false;
            glm::vec2 delta = dragEnd - dragStart;
            Node* root = nullptr;
            if (dragNodeA->fixed) {
                root = dragNodeA;
                while (root->prev) {
                    root = root->prev;
                }
            }
            while (root) {
                root->position[0] += delta.x;
                root->position[1] += delta.y;

                // Important: also move previousPos to match
                root->previousPos[0] += delta.x;
                root->previousPos[1] += delta.y;

                root = root->next;
            }

            dragLine = nullptr;
            dragNodeA = nullptr;

        } else {
            isDragging = false;
        }
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;
    if (isDragging) {
        dragEnd = screenToWorld(window, xpos, ypos);
    }
}

// ---------------------------
// Main
// ---------------------------
int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW\n";
        return -1;
    }

    // Request modern core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    windowPtr = glfwCreateWindow(WIDTH, HEIGHT, "Line Nodes Physics (Modern OpenGL)", nullptr, nullptr);
    if (!windowPtr) {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(windowPtr);
    glfwSwapInterval(1); // vsync

    glewExperimental = GL_TRUE;
    GLenum glewStatus = glewInit();
    if (glewStatus != GLEW_OK) {
        std::cerr << "Failed to init GLEW: " << glewGetErrorString(glewStatus) << std::endl;
        glfwDestroyWindow(windowPtr);
        glfwTerminate();
        return -1;
    }
    srand(time(NULL));

    // ImGui init (OpenGL3 backend only)
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    // Pass GLSL version string to backend (330 core)
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // Create shader
    shaderProgram = createProgram(kVertexShader, kFragmentShader);

    // Framebuffer sizes and projection
    glfwGetWindowSize(windowPtr, &winWidth, &winHeight);
    glfwGetFramebufferSize(windowPtr, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);
    updateProjection();

    // GL state
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Geometry buffers
    createUnitCircle();
    createLineBuffer();

    // Callbacks
    glfwSetFramebufferSizeCallback(windowPtr, framebuffer_size_callback);
    glfwSetMouseButtonCallback(windowPtr, mouse_button_callback);
    glfwSetKeyCallback(windowPtr, key_callback);
    glfwSetCursorPosCallback(windowPtr, cursor_position_callback);
    ImGui_ImplGlfw_InitForOpenGL(windowPtr, true);

    // Initial line (same logic as your original)
    float start[3] = {100.0f, 500.0f, 0.0f};
    Line* line1 = new Line(400, 14, start);
    if (line1->root && line1->root->getNext() && line1->root->getNext()->getNext() &&
        line1->root->getNext()->getNext()->getNext() && line1->root->getNext()->getNext()->getNext()->getNext()) {
        line1->root->getNext()->getNext()->getNext()->getNext()->setFixed(true);
    }
    lines.push_back(line1);

    // Main loop
    while (!glfwWindowShouldClose(windowPtr)) {
        // ImGui new frame
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        // Mode window
        ImGui::Begin("Mode");

        const char* modeName = "Unknown";
        switch (m_Mode) {
            case OPTIONS::DRAGGING:  modeName = "DRAGGING"; break;
            case OPTIONS::INSERTING: modeName = "INSERTING"; break;
            case OPTIONS::DELETING:  modeName = "DELETING"; break;
            case OPTIONS::CUTTING:   modeName = "CUTTING"; break;
            case OPTIONS::TOGGLING:  modeName = "TOGGLING"; break;
        }
        ImGui::Text("Current Mode: %s", modeName);
        ImGui::Checkbox("Paused", &paused);
        if (m_Mode == OPTIONS::INSERTING) {
            ImGui::Separator();
            ImGui::Text("Insert Settings");

            ImGui::SliderFloat("Delta", &gInsertDelta, 5.0f, 200.0f);
            ImGui::SliderInt("Node Count", &gInsertCount, 2, 200);
        }
        ImGui::End();

        glfwGetWindowSize(windowPtr, &winWidth, &winHeight);
        glfwGetFramebufferSize(windowPtr, &fbWidth, &fbHeight);

        glClear(GL_COLOR_BUFFER_BIT);

        // Physics update
        if (!paused) {
            for (Line* line : lines) {
                applyGravity(*line, GRAVITY, DT);
            }

            for (Line* line : lines) {
                std::vector<Node*> nodes;
                Node* curr = line->root;
                while (curr) {
                    nodes.push_back(curr);
                    curr = curr->getNext();
                }
                for (size_t i = 0; i < nodes.size(); ++i) {
                    for (size_t j = i + 2; j < nodes.size(); ++j) {
                        resolveNodeCollision(nodes[i], nodes[j], BALL_RADIUS * 2.0f);
                    }
                }
            }

            for (size_t i = 0; i < lines.size(); ++i) {
                std::vector<Node*> nodesA;
                Node* currA = lines[i]->root;
                while (currA) {
                    nodesA.push_back(currA);
                    currA = currA->getNext();
                }
                for (size_t j = i + 1; j < lines.size(); ++j) {
                    std::vector<Node*> nodesB;
                    Node* currB = lines[j]->root;
                    while (currB) {
                        nodesB.push_back(currB);
                        currB = currB->getNext();
                    }
                    for (Node* nA : nodesA) {
                        for (Node* nB : nodesB) {
                            resolveNodeCollision(nA, nB, BALL_RADIUS * 2.0f);
                        }
                    }
                }
            }

            for (Line* line : lines) {
                Node* curr = line->root;
                while (curr) {
                    enforceWallCollision(curr, BALL_RADIUS);
                    curr = curr->getNext();
                }
            }
        }

        // Render drag preview
        renderDragLine();

        // Render lines + balls
        for (Line* line : lines) {
            renderLine(*line);
        }

        // ImGui render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(windowPtr);
    }

    // Cleanup
    for (Line* line : lines) delete line;

    if (circleVBO) glDeleteBuffers(1, &circleVBO);
    if (circleVAO) glDeleteVertexArrays(1, &circleVAO);

    if (lineVBO) glDeleteBuffers(1, &lineVBO);
    if (lineVAO) glDeleteVertexArrays(1, &lineVAO);

    if (shaderProgram) glDeleteProgram(shaderProgram);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(windowPtr);
    glfwTerminate();
    return 0;
}
