// main.cpp

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <cstring>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "Line.h"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <limits>
#include <string>

#define WIDTH 800
#define HEIGHT 600
#define BALL_QUALITY 20
#define BALL_RADIUS 10.0f
#define DT 0.1f
#define GRAVITY -10.0f
#define DAMPING 0.999f
#define PICK_RADIUS 15.0f

GLuint circleVBO, circleVAO;
GLFWwindow* windowPtr = nullptr;

std::vector<Line*> lines;
bool paused = false;

// Variables for drag-to-create-line
bool isDragging = false;
glm::vec2 dragStart(0.0f);
glm::vec2 dragEnd(0.0f);
Line* dragLine = nullptr;  // line on which drag started
Node* dragNodeA = nullptr;
Node* dragNodeB = nullptr;

// --- Utility functions ---

glm::vec2 screenToWorld(double xpos, double ypos) {
    return glm::vec2((float)xpos, (float)(HEIGHT - ypos));
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

// Find closest segment within maxDist across all lines
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

void insertNewLineBetweenPoints(const glm::vec2& start, const glm::vec2& end, int numPoints) {
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

    // Approximate delta as uniform distance between first two points
    float delta = glm::distance(glm::vec2(newLine->root->position[0], newLine->root->position[1]),
                                glm::vec2(newLine->root->getNext()->position[0], newLine->root->getNext()->position[1]));
    newLine->delta = delta;

    lines.push_back(newLine);
    std::cout << "Created new line with " << numPoints << " nodes.\n";
}

// Collision and physics helpers
void enforceMaxDistance(Node* a, Node* b, float delta) {
    float dir[3];
    float distSq = 0.0f;

    for (int i = 0; i < 3; ++i) {
        dir[i] = b->position[i] - a->position[i];
        distSq += dir[i] * dir[i];
    }

    float dist = sqrt(distSq);
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

    float dist = sqrt(distSq);
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
    else if (node->position[0] > WIDTH - radius)
        node->position[0] = WIDTH - radius;

    if (node->position[1] < radius)
        node->position[1] = radius;
    else if (node->position[1] > HEIGHT - radius)
        node->position[1] = HEIGHT - radius;
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

// --- OpenGL rendering setup and functions ---

void createUnitCircle() {
    std::vector<float> circleVertices = {0.0f, 0.0f};
    for (int i = 0; i <= BALL_QUALITY; ++i) {
        float angle = 2.0f * M_PI * i / BALL_QUALITY;
        circleVertices.push_back(cos(angle));
        circleVertices.push_back(sin(angle));
    }

    glGenVertexArrays(1, &circleVAO);
    glGenBuffers(1, &circleVBO);
    glBindVertexArray(circleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, circleVBO);
    glBufferData(GL_ARRAY_BUFFER, circleVertices.size() * sizeof(float), circleVertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void drawBall(glm::vec2 pos) {
    glPushMatrix();
    glTranslatef(pos.x, pos.y, 0.0f);
    glScalef(BALL_RADIUS, BALL_RADIUS, 1.0f);
    glBindVertexArray(circleVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, BALL_QUALITY + 2);
    glBindVertexArray(0);
    glPopMatrix();
}

void renderLine(Line& line) {
    Node* curr = line.root;

    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINE_STRIP);
    while (curr) {
        glVertex2f(curr->position[0], curr->position[1]);
        curr = curr->getNext();
    }
    glEnd();

    curr = line.root;
    while (curr) {
        if (curr->fixed)
            glColor3f(0.0f, 0.0f, 1.0f); // blue if fixed
        else
            glColor3f(1.0f, 0.0f, 0.0f); // red otherwise

        drawBall(glm::vec2(curr->position[0], curr->position[1]));
        curr = curr->getNext();
    }
}

// --- Input callbacks ---

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    glm::vec2 clickPos = screenToWorld(xpos, ypos);

    if (action == GLFW_PRESS) {
        // On press, check if clicked on a node to toggle fixed
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

        // Else check if near a segment, start drag-to-create-line
        auto hit = findClosestSegmentInAllLines(clickPos, PICK_RADIUS);
        if (hit.line && hit.nodeA && hit.nodeB) {
            isDragging = true;
            dragStart = clickPos;
            dragEnd = clickPos;
            dragLine = hit.line;
            dragNodeA = hit.nodeA;
            dragNodeB = hit.nodeB;
            std::cout << "Started dragging to create new line.\n";
        }
    }
    else if (action == GLFW_RELEASE && isDragging) {
        // On release: finish dragging, create line with user input node count
        isDragging = false;

        // Prompt user for number of nodes (blocking console input)
        int numNodes = 0;
        std::cout << "Enter number of nodes for new line (>=2): ";
        std::cin >> numNodes;
        if (numNodes >= 2) {
            insertNewLineBetweenPoints(dragStart, dragEnd, numNodes);
        } else {
            std::cout << "Invalid node count, canceled.\n";
        }

        dragLine = nullptr;
        dragNodeA = nullptr;
        dragNodeB = nullptr;
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (isDragging) {
        dragEnd = screenToWorld(xpos, ypos);
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        paused = !paused;
        std::cout << (paused ? "Paused" : "Unpaused") << " simulation.\n";
    }
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to init GLFW\n";
        return -1;
    }

    windowPtr = glfwCreateWindow(WIDTH, HEIGHT, "Line Nodes Physics", nullptr, nullptr);
    if (!windowPtr) {
        glfwTerminate();
        std::cerr << "Failed to create window\n";
        return -1;
    }

    glfwMakeContextCurrent(windowPtr);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to init GLEW\n";
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, WIDTH, 0, HEIGHT, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    createUnitCircle();

    glfwSetMouseButtonCallback(windowPtr, mouse_button_callback);
    glfwSetKeyCallback(windowPtr, key_callback);
    glfwSetCursorPosCallback(windowPtr, cursor_position_callback);

    // Initial line setup
    float start[3] = {100.0f, 500.0f, 0.0f};
    Line* line1 = new Line(400, 14, start);
    line1->root->getNext()->getNext()->getNext()->getNext()->setFixed(true);
    lines.push_back(line1);

    while (!glfwWindowShouldClose(windowPtr)) {
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw drag preview line if dragging
        if (isDragging) {
            glColor3f(1.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            glVertex2f(dragStart.x, dragStart.y);
            glVertex2f(dragEnd.x, dragEnd.y);
            glEnd();
        }

        if (!paused) {
            for (Line* line : lines) {
                applyGravity(*line, GRAVITY, DT);
            }

            // Collisions inside lines
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

            // Collisions between lines
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

            // Wall collisions
            for (Line* line : lines) {
                Node* curr = line->root;
                while (curr) {
                    enforceWallCollision(curr, BALL_RADIUS);
                    curr = curr->getNext();
                }
            }
        }

        // Draw all lines
        for (Line* line : lines) {
            renderLine(*line);
        }

        glfwSwapBuffers(windowPtr);
        glfwPollEvents();
    }

    // Cleanup
    for (Line* line : lines)
        delete line;

    glDeleteBuffers(1, &circleVBO);
    glDeleteVertexArrays(1, &circleVAO);
    glfwDestroyWindow(windowPtr);
    glfwTerminate();
    return 0;
}
