#pragma once
// STL
#include <array>
// My includes
#include "Vettore.h"
#include "Utility.h"

//Prototipo classe (necessario perch� richiamata nella classe Box)
template<typename T>
class Diagram;
//Prototipo classe (necessario perch� richiamata nella classe Box)
template<typename T>
class FortuneAlgorithm;


//Classe Box --> Box Rettangolare
template<typename T>
class Box
{
public:
    //Variabili della bounding box quadrata/rettangolare
    T left;     //x-left
    T bottom;   //y-bottom
    T right;    //x-right
    T top;      //y-top

    //Metodo per verificare se un punto � contenuto nella box
    bool contains(const Vector2<T>& point) const
    {
        return almostBetween(point.x, left, right) && almostBetween(point.y, bottom, top);
    }


private:
    //Friendship con altre classi per dar loro accesso a questa sezione private
    friend Diagram<T>;
    friend FortuneAlgorithm<T>;

    //Variabile enum che identifica il lato della box
    enum class Side : int {Left, Bottom, Right, Top};
    //Struct intersezione
    struct Intersection
    {
        Side side;          //lato della box coinvolto nell'intersezione
        Vector2<T> point;   //Punto di intersezione
    };

    //Metodo per capire su quale lato della box avviene la prima intersezione di un vettore con la box
    Intersection getFirstIntersection(const Vector2<T>& origin, const Vector2<T>& direction) const
    {
        //origin must be in the box
        auto intersection = Intersection{};
        auto t = std::numeric_limits<T>::infinity();
        //Caso1: il vettore procede verso x positive
        if (direction.x > static_cast<T>(0.0))
        {
            t = (right - origin.x) / direction.x;   //parametro normalizzatore (di quanto esce il vettore)
            intersection.side = Side::Right;        //lato box coinvolto nell'intersezione
            intersection.point = origin + t * direction;    //punto di intersezione (vedi quaderno)
        }
        //Caso2 : il vettore procede verso x negative
        else if (direction.x < static_cast<T>(0.0))
        {
            t = (left - origin.x) / direction.x;    //parametro normalizzatore (di quanto esce il vettore)
            intersection.side = Side::Left;         //lato box coinvolto nell'intersezione
            intersection.point = origin + t * direction;    //punto di intersezione (vedi quaderno)
        }
        //Caso3: il vettore procede verso y positive
        if (direction.y > static_cast<T>(0.0))
        {   //Check se l'intersezione avviene con lato top anzich� con left
            auto newT = (top - origin.y) / direction.y;
            if (newT < t)   //coordinata y finale ha peso maggiore della x finale
            {
                intersection.side = Side::Top;
                intersection.point = origin + newT * direction; //punto di intersezione (vedi quaderno)
            }
        }
        //Caso4: il vettore procede verso y negative
        else if (direction.y < static_cast<T>(0.0))
        {   //Check se l'intersezione avviene con lato bottom anzich� con right
            auto newT = (bottom - origin.y) / direction.y;
            if (newT < t)   //coordinata y finale ha peso maggiore della x finale
            {
                intersection.side = Side::Bottom;
                intersection.point = origin + newT * direction; //punto di intersezione (vedi quaderno)
            }
        }
        return intersection;
    }

    //Metodo per trovare il numero delle intersezioni del diagramma con la box
    int getIntersections(const Vector2<T>& origin, const Vector2<T>& destination, std::array<Intersection, 2>& intersections) const
    {
        //Se l'intersezione � un corner della box, entrambe le intersezioni sono uguali
        auto direction = destination - origin;  //vettore direzione
        auto t = std::array<T, 2>();            //valori dell'array forniscono una misura della distanza dell'intersezione dall'origine degli assi
        auto i = std::size_t(0); //Indice dell'intersezione corrente
        //Intersezione con side left della box
        if (strictlyLower(origin.x, left) || strictlyLower(destination.x, left))
        {
            t[i] = (left - origin.x) / direction.x;
            if (strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Left;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti y della box
                if (almostBetween(intersections[i].point.y, bottom, top))
                    ++i;    //incremento solo se intersezione � all'interno dei limiti y della box
            }
        }
        //Intersezione con side right della box
        if (strictlyGreater(origin.x, right) || strictlyGreater(destination.x, right))
        {
            t[i] = (right - origin.x) / direction.x;
            if (strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Right;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti y della box
                if (almostBetween(intersections[i].point.y, bottom, top))
                    ++i;    //incremento solo se intersezione � all'interno dei limiti y della box
            }
        }
        //Intersezione con side bottom della box
        if (strictlyLower(origin.y, bottom) || strictlyLower(destination.y, bottom))
        {
            t[i] = (bottom - origin.y) / direction.y;
            if (i < 2 && strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Bottom;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti x della box
                if (almostBetween(intersections[i].point.x, left, right))
                    ++i;    //incremento solo se intersezione � all'interno dei limiti x della box
            }
        }
        //Intersezione con side top della box
        if (strictlyGreater(origin.y, top) || strictlyGreater(destination.y, top))
        {
            t[i] = (top - origin.y) / direction.y;
            if (i < 2 && strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Top;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti x della box
                if (almostBetween(intersections[i].point.x, left, right))
                    ++i;    //incremento solo se intersezione � all'interno dei limiti x della box
            }
        }
        //Ordiniamo le intersezioni dalla pi� vicina alla pi� lontana(swap)
        if (i == 2 && t[0] > t[1])
            std::swap(intersections[0], intersections[1]);
        return i;
    }
};

template<typename T>
// Funzione per intersezione tra due segmenti
std::vector<T> segmentIntersect(Vector2<T> p0, Vector2<T> p1, Vector2<T> p2, Vector2<T> p3)
{
    // Definisco 2 vettori tramite le coordinate dei loro estremi
    Vector2<T> s1 = {(p1.x-p0.x), (p1.y-p0.y)};
    Vector2<T> s2 = {(p3.x-p2.x), (p3.y-p2.y)};

    std::vector<T> intPoint;                        // Punto di intersezione da restituire alla fine (definito come std::vector per accedere alla sua size)

    // Algoritmo di Andre LeMothe (???)
    T s,t;
    s = (s1.y*(p0.x-p2.x) + s1.x*(p0.y-p2.y))/(-s2.x*s1.y + s1.x*s2.y);
    t = (s2.x*(p0.y-p2.y) - s2.y*(p0.x-p2.x))/(-s2.x*s1.y + s1.x*s2.y);

    // std::cout << "========== s: " << s << ", t: " << t << " ==========" << std::endl;
    
    if (s >= -1 && s <= 1 && t >= -1 && t <= 1)
    {
        // Trovata intersezione
        T i_x = p0.x + t*s1.x;
        T i_y = p0.y + t*s1.y;
        intPoint.push_back(i_x);
        intPoint.push_back(i_y);
        // std::cout << "===== SONO QUIIIII " << i_x << ", " << i_y << std::endl;
        // std::cout << p2.x << ", " << p2.y << " --- " << p3.x << ", " << p3.y << std::endl;
    }

    return intPoint;                            // se vuoto vuol dire che non ho intersezioni
}


template<typename T>
// Funzione che restituisce intersezione tra ambiente e ostacolo (-> angolo)
std::vector<T> intersectObstacle(Box<T> RangeBox, Box<T> ObstacleBox)
{
    std::vector<Vector2<T>> AreaVertex;
    std::vector<Vector2<T>> ObstacleVertex;
    std::vector<T> intPoint;            // lo definisco come std::vector per poter accedere alla sua size()

    // Lati della RangeBox
    Vector2<T> alb = {RangeBox.left, RangeBox.bottom};            // bottom left
    Vector2<T> arb = {RangeBox.right, RangeBox.bottom};           // bottom right
    Vector2<T> art = {RangeBox.right, RangeBox.top};              // top right
    Vector2<T> alt = {RangeBox.left, RangeBox.top};               // top left
    AreaVertex.push_back(alb);
    AreaVertex.push_back(arb);
    AreaVertex.push_back(art);
    AreaVertex.push_back(alt);
    AreaVertex.push_back(alb);
    // std::cout << "======== Vertici ambiente: " << std::endl; 
    // for (int i=0; i<AreaVertex.size()-1; i++)
    // {
    //     std::cout << AreaVertex[i].x << ", " << AreaVertex[i].y << std::endl;
    // }

    // Lati dell'ostacolo
    Vector2<T> olb = {ObstacleBox.left, ObstacleBox.bottom};            // bottom left
    Vector2<T> orb = {ObstacleBox.right, ObstacleBox.bottom};           // bottom right
    Vector2<T> ort = {ObstacleBox.right, ObstacleBox.top};              // top right
    Vector2<T> olt = {ObstacleBox.left, ObstacleBox.top};               // top left
    ObstacleVertex.push_back(olb);
    ObstacleVertex.push_back(orb);
    ObstacleVertex.push_back(ort);
    ObstacleVertex.push_back(olt);
    ObstacleVertex.push_back(olb);
    // std::cout << "======== Vertici ostacolo: " << std::endl; 
    // for (int i=0; i<ObstacleVertex.size()-1; i++)
    // {
    //     std::cout << ObstacleVertex[i].x << ", " << ObstacleVertex[i].y << std::endl;
    // }

    // Cerco intersezione tra tutti i lati dell'ambiente e tutti quelli dell'ostacolo
    for (int i=0; i<AreaVertex.size()-1; i++)
    {
        for (int j=0; j<ObstacleVertex.size()-1; j++)
        {
            // Cerco intersezione tra i lati correnti
            std::vector<T> intersection = segmentIntersect(AreaVertex[i], AreaVertex[i+1], ObstacleVertex[j], ObstacleVertex[j+1]);
            if (intersection.size() > 0)
            {
                intPoint.push_back(intersection[0]);            // se trovo intersezione la salvo nella variabile
                intPoint.push_back(intersection[1]);
                // std::cout << "Intersection Found: " << intersection[0] << ", " << intersection[1] << std::endl;
            }
        }
    }

    return intPoint;
}



template<typename T>
// Funzione che restituisce intersezione tra segmento e ostacolo (-> angolo)
bool intersectLineAndObstacle(Vector2<T> p0, Vector2<T> p1, Box<T> ObstacleBox)
{
    std::vector<Vector2<T>> ObstacleVertex;
    std::vector<std::vector<T>> intersection;
    bool doIntersect = false;

    // Lati dell'ostacolo
    Vector2<T> olb = {ObstacleBox.left, ObstacleBox.bottom};            // bottom left
    Vector2<T> orb = {ObstacleBox.right, ObstacleBox.bottom};           // bottom right
    Vector2<T> ort = {ObstacleBox.right, ObstacleBox.top};              // top right
    Vector2<T> olt = {ObstacleBox.left, ObstacleBox.top};               // top left
    ObstacleVertex.push_back(olb);
    ObstacleVertex.push_back(orb);
    ObstacleVertex.push_back(ort);
    ObstacleVertex.push_back(olt);
    ObstacleVertex.push_back(olb);

    // Cerco intersezione tra il segmento e tutti i lati dell'ostacolo
    for (int j=0; j<ObstacleVertex.size()-1; j++)
    {
        std::vector<T> inters = segmentIntersect(p0, p1, ObstacleVertex[j], ObstacleVertex[j+1]);
        if (inters.size() > 0)
        {
            intersection.push_back(inters);
        }
    }

    if (intersection.size() > 1)
        {
            doIntersect = true;
            // std::cout << "========== Corner: " << p1 << "==========" << std::endl;
            // // std::cout << "========== Obstacle num. " << j << std::endl;
            // std::cout << "========== Intersection: \n";
            
            // for (int i=0; i<intersection.size(); i++)
            // {
            //     for (int j=0; j<intersection[i].size(); j=j+2)
            //     {
            //         std::cout << intersection[i][j] << ", " << intersection[i][j+1] << std::endl;
            //     }
                
            // }

        }

    return doIntersect;
}