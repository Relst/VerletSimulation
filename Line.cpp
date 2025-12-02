//
// Created by Karanbir Singh on 11/07/2025.
//

#include "Line.h"
#include <iostream>

Node::Node(float *position, Node *previous)
    : next(nullptr), prev(previous), fixed(false) {
    this->position = new float[3];
    std::memcpy(this->position, position, 3 * sizeof(float));
    this->previousPos = new float[3];
    std::memcpy(this->previousPos, position, 3 * sizeof(float));

    for (int i = 0; i < 3;i++) {
        this->previousPos[i] += 5;
    }

    this->color = new float[3]{1.0f, 1.0f, 1.0f}; // default white
}

Node::~Node() {
    delete[] position;
    delete[] color;
    delete[] previousPos;
}

void Node::setNext(Node *next) {
    this->next = next;
}

Node *Node::getNext() {
    return next;
}

void Node::setPrevious(Node *previous) {
    this->prev = previous;
}

Node *Node::getPrevious() {
    return prev;
}

void Node::setPosition(float *pos) {
    std::memcpy(this->position, pos, 3 * sizeof(float));
}

float *Node::getPosition() {
    return position;
}

void Node::setColor(float *col) {
    std::memcpy(this->color, col, 3 * sizeof(float));
}

float *Node::getColor() {
    return color;
}

void Node::setFixed(bool fixed) {
    this->fixed = fixed;
}

Line::Line() {
    end = nullptr;
    root = nullptr;

}

Line::Line(int size, int numPoints, float *start) {
    float dlt = static_cast<float>(size) / (numPoints - 1);
    initWithDelta(dlt, numPoints, start);
    this->delta = dlt;
}

Line::Line(float delta, int numPoints, float *start) {
    initWithDelta(delta, numPoints, start);
    this->delta = delta;
}

void Line::initWithDelta(float delta, int numPoints, float *start) {
    float pos[3] = {start[0], start[1], start[2]};
    root = new Node(pos, nullptr);
    Node *current = root;

    for (int i = 1; i < numPoints; ++i) {
        pos[0] += delta; // move in +x
        Node *next = new Node(pos, current);
        current->setNext(next);
        current = next;
    }

    end = current;
}

Node *Line::getNode(int idx) {
    Node *current = root;
    int i = 0;
    while (current && i < idx) {
        current = current->getNext();
        i++;
    }
    return current;
}

Line::~Line() {
    Node *current = root;
    while (current) {
        Node *next = current->getNext();
        delete current;
        current = next;
    }
}

void Line::PrintV() {
    Node *current = root;
    int idx = 0;

    while (current) {
        float *pos = current->getPosition();
        std::cout << "| " << idx << " | Pos: ("
                  << pos[0] << ", " << pos[1] << ", " << pos[2] << ")"
                  << (current->getNext() ? " -> " : " [END]") << "\n";
        current = current->getNext();
        ++idx;
    }
}



void Line::Print() {
    Node *current = root;
    int idx = 1;

    while (current) {
        float *pos = current->getPosition();
        std::cout << "| (" << idx << ")  Pos: "<<pos[0] << " |"<< (current->getNext() ? " -> " : " [END]");
        current = current->getNext();
        ++idx;
    }
    std::cout << "\n";
}


void Line::newRoot(Node *newRoot) {
    Node *currRoot = newRoot;
    while (currRoot->getNext()) {
        currRoot = currRoot->getNext();
    }
    root->setPrevious(currRoot);
    currRoot->setNext(root);
    root = newRoot;
}

void Line::newTail(Node *newTail) {
    if (!newTail) return;  // nothing to append

    if (!end) {
        // If the current list is empty, newTail becomes root and end
        root = newTail;

        // Find the tail of newTail chain
        Node *tail = newTail;
        while (tail->getNext()) {
            tail = tail->getNext();
        }
        end = tail;
        return;
    }


    // Connect current end's next to newTail
    end->setNext(newTail);
    newTail->setPrevious(end);

    // Update end pointer to newTail's tail
    Node *tail = newTail;
    while (tail->getNext()) {
        tail = tail->getNext();
    }
    end = tail;
}

std::pair<Line*, Line*> Line::split(int pos) {
    if (pos <= 0) {
        // Return empty first line and the whole line as second
        Line* emptyLine = new Line();
        Line* wholeLine = new Line();
        wholeLine->root = this->root;
        wholeLine->end = this->end;

        // Clear this to avoid double delete
        this->root = nullptr;
        this->end = nullptr;

        return {emptyLine, wholeLine};
    }

    Node* splitNode = getNode(pos);
    if (!splitNode) {
        // pos out of range, return whole line and empty line
        Line* wholeLine = new Line();
        wholeLine->root = this->root;
        wholeLine->end = this->end;

        Line* emptyLine = new Line();

        this->root = nullptr;
        this->end = nullptr;

        return {wholeLine, emptyLine};
    }

    // Disconnect at splitNode
    Node* prevNode = splitNode->getPrevious();
    if (prevNode) {
        prevNode->setNext(nullptr);
    }
    splitNode->setPrevious(nullptr);

    // Build first Line (from root to prevNode)
    Line* firstLine = new Line();
    firstLine->root = this->root;
    firstLine->end = prevNode;

    // Build second Line (from splitNode to old end)
    Line* secondLine = new Line();
    secondLine->root = splitNode;
    secondLine->end = this->end;

    // Clear this line to avoid double free
    this->root = nullptr;
    this->end = nullptr;

    return {firstLine, secondLine};
}



