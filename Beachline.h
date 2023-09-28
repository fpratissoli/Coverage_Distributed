#pragma once
// My includes
#include "Vettore.h"
#include "Diagram.h"
#include "Arc.h"
#include "Utility.h"

// Beachline Class --> RB-Tree Data Structure
template<typename T>
class Beachline
{
public:
    // Beachline/Tree Constructor
    Beachline() : mNil(new Arc<T>), mRoot(mNil) // Arc added as a leaf initially represents the root as well
    {
        mNil->color = Arc<T>::Color::Black;     // Both Nil nodes and the Root node must be black (properties 2, 3)
    }
    // Beachline/Tree Destructor
    ~Beachline()
    {
        free(mRoot);    // Complete tree deletion function starting from the root node
        delete mNil;    // Memory cleanup
    }

    // Remove Copy Constructor
    Beachline(const Beachline&) = delete;
    Beachline& operator=(const Beachline&) = delete;
    // Move Constructor
    Beachline(Beachline&& other)
    {
        mNil = other.mNil;
        mRoot = other.mRoot;
        other.mNil = nullptr;
        other.mRoot = nullptr;
    }
    // Assign Constructor
    Beachline& operator=(Beachline&& other)
    {
        free(mRoot);
        delete mNil;
        mNil = other.mNil;
        mRoot = other.mRoot;
        other.mNil = nullptr;
        other.mRoot = nullptr;
    }

    // Public methods
    // Create a parabola arc given the site
    Arc<T>* createArc(typename Diagram<T>::Site* site, typename Arc<T>::Side side = Arc<T>::Side::Left)
    {   // Create a pointer to a new arc block
        return new Arc<T>{mNil, mNil, mNil, site, nullptr, nullptr, nullptr, mNil, mNil, Arc<T>::Color::Red, side};
    }
    // Check if there is at least one arc/node in the Beachline
    bool isEmpty() const
    {
        return isNil(mRoot); // true: empty tree, the root node itself is a leaf
    }
    // Check if a node/arc is a leaf of the tree
    bool isNil(const Arc<T>* x) const
    {
        return x == mNil;   // true: arc x is a leaf that points to nothing (Nil)
    }
    // Set an arc x as the root
    void setRoot(Arc<T>* x)
    {
        mRoot = x;  // assign the pointers
        mRoot->color = Arc<T>::Color::Black;  // color of the root node: black
    }
    // Get the leftmost arc on the Beachline
    Arc<T>* getLeftmostArc() const
    {
        auto x = mRoot;   // start the search from the root node
        // Traverse all the arcs/nodes by moving to the previous arc of the current arc each time
        while (!isNil(x->prev))
            x = x->prev;
        return x;   // return the leftmost arc
    }
    // Locate the arc on which the new arc associated with the specified site point is inserted, considering the current position of the sweep line (l)
    Arc<T>* locateArcAbove(const Vector2<T>& point, T l) const
    {
        auto node = mRoot;      // start the search from the root node
        auto found = false;     // search boolean flag
        // Continuous search loop until the desired arc is found
        while (!found)
        {
            auto breakpointLeft = -std::numeric_limits<T>::infinity();
            auto breakpointRight = std::numeric_limits<T>::infinity();
            if (!isNil(node->prev)) // if the previous of the Beachline is not a leaf, calculate the x-coordinate of intersection between them
                breakpointLeft = computeBreakpoint(node->prev->site->point, node->site->point, l, node->prev->side);
            if (!isNil(node->next)) // if the next of the Beachline is not a leaf, calculate the x-coordinate of intersection between them
                breakpointRight = computeBreakpoint(node->site->point, node->next->site->point, l, node->next->side);
            if (point.x < breakpointLeft)
                node = node->left;  // move to the leftchild node
            else if (point.x > breakpointRight)
                node = node->right; // move to the rightchild node
            else
                found = true;   // node found
        }
        return node;
    }
    // Insert an arc y before an arc x
    void insertBefore(Arc<T>* x, Arc<T>* y)
    {
        // Step 1: Determine the position to insert node y
        // There is no left child of arc x
        if (isNil(x->left))
        {
            x->left = y;
            y->parent = x;
        }
        // There is already a left child of arc x
        else
        {
            x->prev->right = y;
            y->parent = x->prev;
        }
        // Step 2: Set DCEL pointers
        y->prev = x->prev;
        if (!isNil(y->prev))
            y->prev->next = y;
        y->next = x;
        x->prev = y;
        // Step 3: Rebalance the tree
        insertFixup(y);
    }
    // Insert an arc y after an arc x
    void insertAfter(Arc<T>* x, Arc<T>* y)
    {
        // Step 1: Determine the position to insert node y
        // There is no right child of arc x
        if (isNil(x->right))
        {
            x->right = y;
            y->parent = x;
        }
        // There is already a right child of arc x
        else
        {
            x->next->left = y;
            y->parent = x->next;
        }
        // Step 2: Set DCEL pointers
        y->next = x->next;
        if (!isNil(y->next))
            y->next->prev = y;
        y->prev = x;
        x->next = y;
        // Step 3: Rebalance the tree
        insertFixup(y);
    }
    // Replace an arc/node x with another arc/node y: so when an arc is replaced by a middleArc, the middleArc becomes the parent of leftArc and rightArc
    void replace(Arc<T>* x, Arc<T>* y)
    {
        transplant(x, y);           // transplant subtree rooted at x with subtree rooted at y
        y->left = x->left;          // y inherits the left child of x
        y->right = x->right;        // y inherits the right child of x
        if (!isNil(y->left))
            y->left->parent = y;    // attach the left child to y in case x has one
        if (!isNil(y->right))
            y->right->parent = y;   // attach the right child to y in case x has one

        y->prev = x->prev;          // y inherits the previous pointer of x
        y->next = x->next;          // y inherits the next pointer of x
        if (!isNil(y->prev))
            y->prev->next = y;      // attach the previous arc to y in case x has one
        if (!isNil(y->next))
            y->next->prev = y;      // attach the next arc to y in case x has one
        //y->side = x->side;          // y inherits the side of x
        //y->site = x->site;          // y inherits the site of x
        y->color = x->color;        // y inherits the color of x
        //delete x;                   // delete arc x (memory cleanup)
    }
    // Remove an arc/node from the Beachline
    void remove(Arc<T>* z)
    {
        auto y = z;
        auto yOriginalColor = y->color;
        auto x = mNil;

        if (isNil(z->left))
        {
            x = z->right;
            transplant(z, z->right);
        }

        else if (isNil(z->right))
        {
            x = z->left;
            transplant(z, z->left);
        }

        else
        {
            y = minimum(z->right);
            yOriginalColor = y->color;
            x = y->right;

            if (y->parent == z)
                x->parent = y;

            else
            {
                transplant(y, y->right);
                y->right = z->right;
                y->right->parent = y;
            }

            transplant(z, y);
            y->left = z->left;
            y->left->parent = y;
            y->color = z->color;
        }

        if (yOriginalColor == Arc<T>::Color::Black)
            removeFixup(x);

        if (!isNil(z->prev))
            z->prev->next = z->next;
        if (!isNil(z->next))
            z->next->prev = z->prev;
    }

    std::ostream& print(std::ostream& os) const
    {
        std::cout << "Root: ";
        return printArc(os, mRoot);
//        auto arc = getLeftmostArc();
//        while (!isNil(arc))
//        {
//            os << arc->site->index << ' ';
//            arc = arc->next;
//        }
//        return os;
    }

private:
    // private members
    Arc<T>* mNil;   // Nil leaf
    Arc<T>* mRoot;  // Root arc


    Arc<T>* minimum(Arc<T>* x) const
    {
        while (!isNil(x->left))
            x = x->left;
        return x;
    }

    void transplant(Arc<T>* u, Arc<T>* v)
    {
        if (isNil(u->parent))
            mRoot = v;
        else if (u == u->parent->left)
            u->parent->left = v;
        else
            u->parent->right = v;

        v->parent = u->parent;
    }

    // Tree rebalancing after the insertion of a new arc/node (red).
    void insertFixup(Arc<T>* z)
    {
        while (z->parent->color == Arc<T>::Color::Red)
        {
            if (z->parent == z->parent->parent->left)
            {
                auto y = z->parent->parent->right;
                //Case 1: y is red
                if (y->color == Arc<T>::Color::Red)
                {   //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    y->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    z = z->parent->parent;
                }
                //y is black
                else
                {
                    //Case 2: z is a rightchild
                    if (z == z->parent->right)
                    {
                        z = z->parent;
                        leftRotate(z);
                    }
                    //Case 3: z is now a leftchild
                    //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    rightRotate(z->parent->parent); //right-rotation
                }
            }
            else
            {
                auto y = z->parent->parent->left;
                //Case 1: y is red
                if (y->color == Arc<T>::Color::Red)
                {   //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    y->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    z = z->parent->parent;
                }
                //y is black
                else
                {
                    //Case 2: z is a leftchild
                    if (z == z->parent->left)
                    {
                        z = z->parent;
                        rightRotate(z); //right-rotation
                    }
                    //Case 3: z is now a rightchild
                    //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    leftRotate(z->parent->parent);  //left-rotation
                }
            }
        }
        mRoot->color = Arc<T>::Color::Black;
    }
    // Tree rebalancing following the removal of an arc/node.
    void removeFixup(Arc<T>* x)
    {
        while (x != mRoot && x->color == Arc<T>::Color::Black)
        {
            if (x == x->parent->left)
            {
                auto w = x->parent->right;
                //Case 1
                if (w->color == Arc<T>::Color::Red)
                {
                    w->color = Arc<T>::Color::Black;
                    x->parent->color = Arc<T>::Color::Red;
                    leftRotate(x->parent);
                    w = x->parent->right;
                }
                //Case 2
                if (w->left->color == Arc<T>::Color::Black && w->right->color == Arc<T>::Color::Black)
                {
                    w->color = Arc<T>::Color::Red;
                    x = x->parent;
                }
                else
                {
                    //Case 3
                    if (w->right->color == Arc<T>::Color::Black)
                    {
                        w->left->color = Arc<T>::Color::Black;
                        w->color = Arc<T>::Color::Red;
                        rightRotate(w); //right-rotation
                        w = x->parent->right;
                    }
                    //Case 4
                    //Recolor
                    w->color = x->parent->color;
                    x->parent->color = Arc<T>::Color::Black;
                    w->right->color = Arc<T>::Color::Black;
                    leftRotate(x->parent);  //left-rotation
                    x = mRoot;
                }
            }
            else
            {
                auto w = x->parent->left;
                //Case 1
                if (w->color == Arc<T>::Color::Red)
                {
                    w->color = Arc<T>::Color::Black;
                    x->parent->color = Arc<T>::Color::Red;
                    rightRotate(x->parent);
                    w = x->parent->left;
                }
                //Case 2
                if (w->left->color == Arc<T>::Color::Black && w->right->color == Arc<T>::Color::Black)
                {
                    w->color = Arc<T>::Color::Red;
                    x = x->parent;
                }
                else
                {
                    //Case 3
                    if (w->left->color == Arc<T>::Color::Black)
                    {
                        w->right->color = Arc<T>::Color::Black;
                        w->color = Arc<T>::Color::Red;
                        leftRotate(w);
                        w = x->parent->left;
                    }
                    // Case 4
                    w->color = x->parent->color;
                    x->parent->color = Arc<T>::Color::Black;
                    w->left->color = Arc<T>::Color::Black;
                    rightRotate(x->parent);
                    x = mRoot;
                }
            }
        }
        x->color = Arc<T>::Color::Black;
    }

    void leftRotate(Arc<T>* x)
    {
        auto y = x->right;
        x->right = y->left;
        if (!isNil(y->left))
            y->left->parent = x;
        y->parent = x->parent;
        if (isNil(x->parent))
            mRoot = y;
        else if (x->parent->left == x)
            x->parent->left = y;
        else
            x->parent->right = y;
        y->left = x;
        x->parent = y;
    }

    void rightRotate(Arc<T>* y)
    {
        auto x = y->left;
        y->left = x->right;
        if (!isNil(x->right))
            x->right->parent = y;
        x->parent = y->parent;
        if (isNil(y->parent))
            mRoot = x;
        else if (y->parent->left == y)
            y->parent->left = x;
        else
            y->parent->right = x;
        x->right = y;
        y->parent = x;
    }
    // Calculating the x-coordinate of intersection/breakpoint between two arcs, given the associated sites point1 and point2, and the position y of the sweepline (l).
    T computeBreakpoint(const Vector2<T>& point1, const Vector2<T>& point2, T l, typename Arc<T>::Side side) const
    {
        auto x1 = point1.x, y1 = point1.y, x2 = point2.x, y2 = point2.y;
        // If the two arcs have the same curvature (because the y-coordinates of the sites are approximately equal)
        if (almostEqual(y1, y2))
        {
            // Case 1. The intersection breakpoint is found halfway between the x-coordinates of the two points.
            if (x1 < x2)
                return (x1 + x2) / 2;
            // Case 2. The intersection breakpoint is at infinity (-inf or +inf depending on the side of the arc).
            else
                return side == Arc<T>::Side::Left ?
                    -std::numeric_limits<T>::infinity() :
                    std::numeric_limits<T>::infinity();
        }

        if (almostEqual(y1, l))
            return x1;
        if (almostEqual(y2, l))
            return x2;

        auto d1 = 1.0 / (2.0 * (y1 - l));
        auto d2 = 1.0 / (2.0 * (y2 - l));
        auto a = d1 - d2;
        auto b = 2.0 * (x2 * d2 - x1 * d1);
        auto c = (y1 * y1 + x1 * x1 - l * l) * d1 - (y2 * y2 + x2 * x2 - l * l) * d2;
        auto delta = b * b - 4.0 * a * c;
        return (-b + std::sqrt(delta)) / (2.0 * a);
    }
    // Deletion of the subtree rooted at node x (works in conjunction with the destructor).
    void free(Arc<T>* x)
    {
        if (isNil(x))
            return;
        else
        {
            free(x->left);
            free(x->right);
            delete x;
        }
    }

    std::ostream& printArc(std::ostream& os, const Arc<T>* arc, std::string tabs = "") const
    {
        os << arc->site->index << std::endl;
        if (!isNil(arc->left)){
            std::cout << "Leftchild di " << arc->site->index << ": ";
            printArc(os, arc->left);
        }
        if (!isNil(arc->right)){
            std::cout << "Rightchild di " << arc->site->index << ": ";
            printArc(os, arc->right);
        }
        return os;
    }
};

// Overloading the stream operator for a Beachline object for printing.
template<typename T>
std::ostream& operator<<(std::ostream& os, const Beachline<T>& beachline)
{
    return beachline.print(os);
}
