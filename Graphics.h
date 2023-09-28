#pragma once

// STL
#include <iostream>
#include <vector>
// SFML
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
// My includes
#include "Voronoi.h"
#include "Vettore.h"

//Color points/sites
const uint8_t P_RED = 100;    //[0-255]
const uint8_t P_GREEN = 250;  //[0-255]
const uint8_t P_BLUE = 50;    //[0-255]
//Color lines/edges
const uint8_t L_RED = 218;  //[0-255]
const uint8_t L_GREEN = 5;  //[0-255]
const uint8_t L_BLUE = 5;  //[0-255]
//Color cross/prev_points
const uint8_t C_RED = 255;  //[0-255]
const uint8_t C_GREEN = 255;  //[0-255]
const uint8_t C_BLUE = 255;  //[0-255]
//Color gaussians
const uint8_t G_RED = 153;    //[0-255]
const uint8_t G_GREEN = 0;  //[0-255]
const uint8_t G_BLUE = 153;    //[0-255]

class Graphics
{
private:
    double AREA_LEFT { -0.5 };
    double AREA_BOTTOM { -0.3 };
    double AREA_SIZE_x { 5.0 };
    double AREA_SIZE_y { 5.0 };
    double VAR { 1 };

    double AREA_SIZE = std::min(AREA_SIZE_x, AREA_SIZE_y);

    //Window parameters
    double WINDOW_WIDTH { AREA_SIZE_x*75.0 };    //600.0;
    double WINDOW_HEIGHT { AREA_SIZE_y*75.0 };
    double POINT_RADIUS { AREA_SIZE/200 };
    double OFFSET { 1 };

public:
    unsigned int num = 0;               //intero per gestione grafica diagrammi
    bool decentralized_on = false;      //gestione visualizzazione tipologia diagramma

    //graphics window
    //Rendering with SFML
    sf::ContextSettings settings;
    std::unique_ptr<sf::RenderWindow> window;
    std::unique_ptr<sf::View> view;

    Graphics(){};   //default constructor
    //Graphics(double AREA_SIZE, double POINT_RADIUS, double OFFSET, double VAR){
    //    this->AREA_SIZE = AREA_SIZE;
    //    this->POINT_RADIUS = POINT_RADIUS;
    //    this->OFFSET = OFFSET;
    //    this->VAR = VAR;
    //};
    Graphics(double AREA_SIZE_x_, double AREA_SIZE_y_, double AREA_LEFT_, double AREA_BOTTOM_, double VAR_)          //constructor
        : AREA_SIZE_x(AREA_SIZE_x_), AREA_SIZE_y(AREA_SIZE_y_), AREA_LEFT(AREA_LEFT_), AREA_BOTTOM(AREA_BOTTOM_), VAR(VAR_)
        {
            //Rendering with SFML
            settings.antialiasingLevel = 8; //anti-aliasing
            //Create RenderWindow
            WINDOW_WIDTH = WINDOW_WIDTH/AREA_SIZE_x*8;
            WINDOW_HEIGHT = WINDOW_HEIGHT/AREA_SIZE_y*8;
            window.reset(new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Voronoi Diagram", sf::Style::Default, settings));
            view.reset(new sf::View(sf::FloatRect(-0.025 + AREA_LEFT, -0.025 - AREA_BOTTOM, AREA_SIZE_x+0.05, AREA_SIZE_y+0.05)));  //OK, parametrizzato (- per y e non + poiché il sistema di riferimento SFML per y è opposto)

            window->setView(*view);
            //Set FramerateLimit per Lloyd's algorithm
            window->setFramerateLimit(60);
        };

    ~Graphics(){
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
        if (this->isOpen())
        {
            this->close();
        }
    };


    //Rendering functions
    //Drawing Origin Reference --> visualizza il sistema di riferimento globale (FISSO)
    void drawGlobalReference(sf::Color color_point, sf::Color color_lines){
        if(!decentralized_on){
            //Punto origine
            sf::CircleShape origin(POINT_RADIUS);
            origin.setPosition(sf::Vector2f(0.0 - POINT_RADIUS, AREA_SIZE_y - 0.0 - POINT_RADIUS));
            origin.setFillColor(color_point);
            this->window->draw(origin);
            //Asse X
            sf::Vertex x_axis[] =
            {
                sf::Vertex(sf::Vector2f(0.0, AREA_SIZE_y - 0.0), color_lines),
                sf::Vertex(sf::Vector2f(AREA_SIZE_x/20, AREA_SIZE_y - 0.0), color_lines)
            };
            //Asse Y
            sf::Vertex y_axis[] =
            {
                sf::Vertex(sf::Vector2f(0.0, AREA_SIZE_y - 0.0), color_lines),
                sf::Vertex(sf::Vector2f(0.0, AREA_SIZE_y - AREA_SIZE_y/20), color_lines)
            };
            this->window->draw(x_axis, 2, sf::Lines);
            this->window->draw(y_axis, 2, sf::Lines);
        }
    }

    //Drawing cross-point
    template<typename T>
    void drawCrossCentralized(Vector2<T> point, sf::Color color){
        //Visualizzazione forma a croce
        sf::Vertex point_hline[] =
        {
            sf::Vertex(sf::Vector2f(point.x - POINT_RADIUS, AREA_SIZE_y - point.y), color),
            sf::Vertex(sf::Vector2f(point.x + POINT_RADIUS, AREA_SIZE_y - point.y), color)
        };
        sf::Vertex point_vline[] =
        {
            sf::Vertex(sf::Vector2f(point.x, AREA_SIZE_y - (point.y-POINT_RADIUS)), color),
            sf::Vertex(sf::Vector2f(point.x, AREA_SIZE_y - (point.y+POINT_RADIUS)), color)
        };
        //Comando draw per disegnare effettivamente la linea nella finestra
        this->window->draw(point_hline, 2, sf::Lines);
        this->window->draw(point_vline, 2, sf::Lines);
    }

    //Drawing cross-point
    template<typename T>
    void drawCrossDecentralized(Vector2<T> point, sf::Color color, const std::vector<Diagram<T>>& diagrams){
        //Visualizzazione forma a croce
        sf::Vertex point_hline[] =
        {
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + point.x - POINT_RADIUS, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - point.y), color),
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + point.x + POINT_RADIUS, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - point.y), color)
        };
        sf::Vertex point_vline[] =
        {
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + point.x, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - (point.y-POINT_RADIUS)), color),
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + point.x, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - (point.y+POINT_RADIUS)), color)
        };
        //Comando draw per disegnare effettivamente la linea nella finestra
        this->window->draw(point_hline, 2, sf::Lines);
        this->window->draw(point_vline, 2, sf::Lines);
    }

    //Drawing centroid history --> visualizzazione storico dei centroidi (on centralized mode)
    template<typename T>
    void drawHistory(const std::vector<Vector2<T>>& prev_global_points, unsigned int NbPoints){
        //Storico dei centroidi sul diagramma globale
        for(std::size_t i=0; i<prev_global_points.size(); ++i){
            if(i%NbPoints==0)
                drawCrossCentralized(prev_global_points[i], sf::Color(C_RED, C_GREEN, C_BLUE));
            else if(i%NbPoints==1)
                drawCrossCentralized(prev_global_points[i], sf::Color(C_RED, C_GREEN, 0));
            else if(i%NbPoints==2)
                drawCrossCentralized(prev_global_points[i], sf::Color(C_RED, 0, C_BLUE));
            else if(i%NbPoints==3)
                drawCrossCentralized(prev_global_points[i], sf::Color(0, C_GREEN, C_BLUE));
            else if(i%NbPoints==4)
                drawCrossCentralized(prev_global_points[i], sf::Color(150, 150, C_BLUE));
            else if(i%NbPoints==5)
                drawCrossCentralized(prev_global_points[i], sf::Color(C_RED, 150, 150));
            else if(i%NbPoints==6)
                drawCrossCentralized(prev_global_points[i], sf::Color(150, 150, 150));
        }
    }

    //Drawing next centroid --> on decentralized mode
    template<typename T>
    void drawNextCentroid(const std::vector<Vector2<T>>& local_centroids, const std::vector<Diagram<T>>& diagrams){
        //Centroide successivo cella del robot in ogni diagramma decentralizzato
        drawCrossDecentralized(local_centroids[num], sf::Color(C_RED, C_GREEN, C_BLUE), diagrams);
    }

    //Drawing a Point --> CircleShape with POINT_RADIUS as radius parameter
    template<typename T>
    void drawPoint(Vector2<T> site, sf::Color color, const std::vector<Diagram<T>>& diagrams){
        if(!decentralized_on){
            sf::CircleShape point(POINT_RADIUS);
            point.setPosition(sf::Vector2f(site.x - POINT_RADIUS, AREA_SIZE_y - site.y - POINT_RADIUS));
            //Settiamo il colore del punto
            point.setFillColor(color);
            //Comando draw per disegnare effettivamente il punto nella finestra
            this->window->draw(point);
        }
        else{
            sf::CircleShape point(POINT_RADIUS);
            point.setPosition(sf::Vector2f(site.x + diagrams[num].getGlobalPoint().x - POINT_RADIUS,
                                           AREA_SIZE_y - diagrams[num].getGlobalPoint().y - site.y - POINT_RADIUS));
            //Settiamo il colore del punto
            point.setFillColor(color);
            //Comando draw per disegnare effettivamente il punto nella finestra
            this->window->draw(point);
        }
    }

    //Drawing Multiple Points --> ciclo for di più drawPoint functions
    template<typename T>
    void drawPoints(const Diagram<T> &diagram, const std::vector<Diagram<T>>& diagrams){
        for(std::size_t i = 0; i < diagram.getNbSites(); ++i){
            if(diagram.getSite(i)->index == 0 && decentralized_on){
                //Sito centrale azzurro (sito proprietario del diagramma)
                drawPoint(diagram.getSite(0)->point, sf::Color(0,200,255), diagrams);
            }
            else{
                drawPoint(diagram.getSite(i)->point, sf::Color(P_RED, P_GREEN, P_BLUE), diagrams);
            }
        }
    }

    template<typename T>
    void drawPoint(Vector2<T> site, sf::Color color){
        sf::CircleShape point(POINT_RADIUS);
        point.setPosition(sf::Vector2f(site.x - POINT_RADIUS, AREA_SIZE_y - site.y - POINT_RADIUS));
        //Settiamo il colore del punto
        point.setFillColor(color);
        //Comando draw per disegnare effettivamente il punto nella finestra
        this->window->draw(point);
    }

    //Drawing Multiple Points --> ciclo for di più drawPoint functions
    template<typename T>
    void drawPoints(const Diagram<T> &diagram){
        for(std::size_t i = 0; i < diagram.getNbSites(); ++i){
            drawPoint(diagram.getSite(i)->point, sf::Color(P_RED, P_GREEN, P_BLUE));
        }
    }

    // Draw Gaussian
    template<typename T>
    void drawGauss(Vector2<T> mean, sf::Color color){
        sf::CircleShape point(POINT_RADIUS);     // Definizione figura triangolare
        point.setPosition(sf::Vector2f(mean.x - POINT_RADIUS, AREA_SIZE_y - mean.y - POINT_RADIUS));
        //Settiamo il triangolo del punto
        point.setFillColor(color);
        //Comando draw per disegnare effettivamente il triangolo nella finestra
        this->window->draw(point);
    }

    //Drawing Multiple Gaussians --> ciclo for di più drawGauss functions
    template<typename T>
    void drawMultiGauss(std::vector<Vector2<T>> MEANs){
        for(std::size_t i = 0; i < MEANs.size(); ++i){
            drawGauss(MEANs[i], sf::Color(C_RED, C_GREEN, C_BLUE));
        }
    }

    //Drawing an Edge --> Edge disegnato usando la primitiva/vertex array line a 2 vertici
    template<typename T>
    void drawEdge_global(Vector2<T> origin, Vector2<T> destination, sf::Color color){
        //Usiamo la line without thickness
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(origin.x, AREA_SIZE_y - origin.y), color),
            sf::Vertex(sf::Vector2f(destination.x, AREA_SIZE_y - destination.y), color)
        };
        //Comando draw per disegnare effettivamente la linea nella finestra
        this->window->draw(line, 2, sf::Lines);
    }

    //Drawing an Edge --> Edge disegnato usando la primitiva/vertex array line a 2 vertici
    template<typename T>
    void drawEdge_local(Vector2<T> origin, Vector2<T> destination, sf::Color color, const std::vector<Diagram<T>>& diagrams){
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + origin.x, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - origin.y), color),
            sf::Vertex(sf::Vector2f(diagrams[num].getGlobalPoint().x + destination.x, AREA_SIZE_y - diagrams[num].getGlobalPoint().y - destination.y), color)
        };
        //Comando draw per disegnare effettivamente la linea nella finestra
        this->window->draw(line, 2, sf::Lines);
    }

    //Drawing an Edge --> Edge disegnato usando la primitiva/vertex array line a 2 vertici
    template<typename T>
    void drawEdge(Vector2<T> origin, Vector2<T> destination, sf::Color color, const std::vector<Diagram<T>>& diagrams){
        //Usiamo la line without thickness
        if(!decentralized_on){
            drawEdge_global(origin, destination, color);
        }
        else{
            drawEdge_local(origin, destination, color, diagrams);
        }
    }

    //Drawing Edges/Diagram
    template<typename T>
    void drawDiagram(const Diagram<T> &diagram, const std::vector<Diagram<T>>& diagrams){
        //Per tutte le facce/celle di Voronoi
        for(const auto& site : diagram.getSites()){
            Vector2<T> center = site.point;
            auto face = site.face;
            auto halfEdge = face->outerComponent;   //ottieni l'half-edge associato
            if(halfEdge == nullptr){    //per sicurezza
                continue;
            }
            //Scansiono tutti gli half-edge della faccia finchè non ritorno all'outerComponent
            while(halfEdge->prev != nullptr){   //Check della chiusura della cella
                halfEdge = halfEdge->prev;
                if(halfEdge == face->outerComponent){
                    break;  //break dal while ovvero proseguo oltre (loop altrimenti è infinito)
                }
            }
            auto start = halfEdge;  //segno come start l'halfedge outerComponent
            //Scansiono tutti gli half-edge della faccia finchè non ritorno all'outerComponent/start
            while(halfEdge != nullptr){ //Loop di disegno del diagramma
                if(halfEdge->origin != nullptr && halfEdge->destination != nullptr){
                    //Inserisco OFFSET e disegno i lati
                    auto origin = (halfEdge->origin->point - center)*OFFSET + center;
                    auto destination = (halfEdge->destination->point - center)*OFFSET + center;
                    drawEdge(origin, destination, sf::Color(L_RED, L_GREEN, L_BLUE), diagrams);
                }
                //Passo all'half-edge successivo
                halfEdge = halfEdge->next;
                if(halfEdge == start){  //Loop di disegno termina quando all'half-edge di start
                    break;  //break del while ma si prosegue con il for nella nuova cella
                }
            }
        }
    }

    //Drawing Edges/Diagram
    template<typename T>
    void drawDiagram(const Diagram<T> &diagram){
        //Per tutte le facce/celle di Voronoi
        for(const auto& site : diagram.getSites()){
            Vector2<T> center = site.point;
            auto face = site.face;
            auto halfEdge = face->outerComponent;   //ottieni l'half-edge associato
            if(halfEdge == nullptr){    //per sicurezza
                continue;
            }
            //Scansiono tutti gli half-edge della faccia finchè non ritorno all'outerComponent
            while(halfEdge->prev != nullptr){   //Check della chiusura della cella
                halfEdge = halfEdge->prev;
                if(halfEdge == face->outerComponent){
                    break;  //break dal while ovvero proseguo oltre (loop altrimenti è infinito)
                }
            }
            auto start = halfEdge;  //segno come start l'halfedge outerComponent
            //Scansiono tutti gli half-edge della faccia finchè non ritorno all'outerComponent/start
            while(halfEdge != nullptr){ //Loop di disegno del diagramma
                if(halfEdge->origin != nullptr && halfEdge->destination != nullptr){
                    //Inserisco OFFSET e disegno i lati
                    auto origin = (halfEdge->origin->point - center)*OFFSET + center;
                    auto destination = (halfEdge->destination->point - center)*OFFSET + center;
                    drawEdge_global(origin, destination, sf::Color(L_RED, L_GREEN, L_BLUE));
                }
                //Passo all'half-edge successivo
                halfEdge = halfEdge->next;
                if(halfEdge == start){  //Loop di disegno termina quando all'half-edge di start
                    break;  //break del while ma si prosegue con il for nella nuova cella
                }
            }
        }
    }

    template<typename T>
    void drawGaussianContour(const Vector2<T>& p_t, const T var, unsigned int NbVarianceCircles){
        if(!decentralized_on){
            //Punto di interesse p_t
            sf::CircleShape circle(0.75*POINT_RADIUS);
            //Colore Giallo chiaro
            circle.setFillColor(sf::Color(255,255,102));
            //Position x,y
            circle.setPosition(p_t.x - 0.75*POINT_RADIUS, AREA_SIZE_y - p_t.y - 0.75*POINT_RADIUS);

            this->window->draw(circle);
            //Cerchio esterno concentrico al punto p_t
            for(std::size_t i=1; i<=NbVarianceCircles; ++i){
                //Diametro = VAR
                sf::CircleShape variance_circle(VAR*i);
                //Colore trasparente
                variance_circle.setFillColor(sf::Color::Transparent);
                //External Outline Spessore
                 variance_circle.setOutlineThickness(0.005);
                //External Outline Colore (Grigio)
                variance_circle.setOutlineColor(sf::Color(210,210,210));
                //Position center x,y
                variance_circle.setPosition(p_t.x - VAR*i, AREA_SIZE_y - p_t.y - VAR*i);

                this->window->draw(variance_circle);
            }
        }
    }

    template<typename T>
    void drawGaussianContours(const std::vector<Vector2<T>>& p_ts, const std::vector<T> vars){
        if(!decentralized_on){
            for(std::size_t i=0; i<p_ts.size(); ++i){
                drawGaussianContour(p_ts[i], vars[i], 3);
            }
        }
    }

    // Funzione per disegnare un ostacolo
    template<typename T>
    void drawObstacle(Box<T> ObstacleBox, sf::Color color){
        sf::Vertex vertices[] =
            {
            sf::Vertex(sf::Vector2f(ObstacleBox.left, AREA_SIZE_y - ObstacleBox.bottom), sf::Color::Red),
            sf::Vertex(sf::Vector2f(ObstacleBox.right, AREA_SIZE_y - ObstacleBox.bottom), sf::Color::Red),
            sf::Vertex(sf::Vector2f(ObstacleBox.right, AREA_SIZE_y - ObstacleBox.top), sf::Color::Red),
            sf::Vertex(sf::Vector2f(ObstacleBox.left, AREA_SIZE_y - ObstacleBox.top), sf::Color::Red)
            };
        // draw it
        this->window->draw(vertices, 4, sf::Quads);
    }

    bool isOpen(){
        return this->window->isOpen();
    }

    void close(){
        this->window->close();
    }

    void clear(){
        this->window->clear(sf::Color::Black);
    }

    void display(){
        this->window->display();
    }
};
