#include "../includes/edge.hpp"

#include <iostream>

Edge::Edge(Node* head, Node* tail, int weight) {
    this->head = head;
    this->tail = tail;
    this->weight = weight;
}

Edge::~Edge() {
    this->head = nullptr;
    this->tail = nullptr;
    this->weight = 0;
}

Node* Edge::getHead() {
    return this->head;
}

Node* Edge::getTail() {
    return this->tail;
}

int Edge::getWeight() {
    return this->weight;
}

void Edge::setHead(Node* head) {
    this->head = head;
}

void Edge::setTail(Node* tail) {
    this->tail = tail;
}

void Edge::setWeight(int weight) {
    this->weight = weight;
}
