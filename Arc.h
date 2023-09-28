#pragma once
// My includes
#include "Diagram.h"

// Class prototype (necessary because it's referenced in the Arc class)
template<typename T>
class Event;

// Arc Class - Represents a parabolic arc of the Beachline, each Arc object is a node in the RB-Tree that represents the Beachline
template<typename T>
struct Arc
{
    // Node color (for balancing the tree)
    enum class Color{Red, Black};
    // Side of the arc (relative to an imaginary vertical line)
    enum class Side{Left, Right};

    // Hierarchy of arcs: pointers to other nodes/arcs in the RB-Tree
    Arc<T>* parent;     // Pointer to the parent node
    Arc<T>* left;       // Pointer to the left child node
    Arc<T>* right;      // Pointer to the right child node

    // Associations with the Diagram: pointers to the associated site and the half-edges on the left and right sides of the arc
    typename Diagram<T>::Site* site;                // Pointer to the site associated with the arc
    typename Diagram<T>::HalfEdge* leftHalfEdge;    // Pointer to the half-edge on the left side of the arc
    typename Diagram<T>::HalfEdge* rightHalfEdge;   // Pointer to the half-edge on the right side of the arc
    Event<T>* event;                                // Pointer to the event associated with the arc

    // DCEL (Doubly Connected Edge List) Optimization: pointers to the next and previous arcs in the Beachline (as if the Beachline were a DCEL)
    Arc<T>* prev;               // Pointer to the previous arc in the Beachline
    Arc<T>* next;               // Pointer to the next arc in the Beachline

    // Red-Black Tree balancing for improved efficiency in search, insert, and delete operations
    // The tree is balanced when, for any node, the number of nodes in the left subtree differs by at most 1 from the number of nodes in the right subtree (this ensures uniform time complexity for any operation on the tree)
    Color color;    // Tree balancing is achieved using colors

    // To determine if the arc goes to -infinity or +infinity (useful for calculating intersection points between adjacent arcs on the Beachline)
    Side side;
};
