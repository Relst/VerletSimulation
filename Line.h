//
// Created by Karanbir Singh on 11/07/2025.
//

#ifndef LINE_H
#define LINE_H
#include <__utility/pair.h>

class Node {
public:
  float *position;
  float *previousPos;
  float *color;
  Node *next;
  Node *prev;
  bool fixed;

  Node(float *position, Node *previous = nullptr);
  ~Node();

  void setNext(Node *next);
  Node *getNext();

  void setPrevious(Node *previous);
  Node *getPrevious();

  void setPosition(float *position);
  float *getPosition();

  void setColor(float *color);
  float *getColor();

  void setFixed(bool fixed);
};


class Line {
public:
  Node *root;
  Node *end;
  float delta;
  Line();

  Line(int size, int numPoints, float *start);
  Line(float delta, int numPoints, float *start);
  ~Line();

  Node *getNode(int idx);

  void PrintV();
  void Print();

  void newRoot(Node *root);

  void newTail(Node *newTail);

  std::pair<Line*, Line*> split(int pos);

private:
  void initWithDelta(float delta, int numPoints, float *start);
};


#endif //LINE_H
