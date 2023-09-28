// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <math.h>
// SFML
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
// My includes
#include "FortuneAlgorithm.h"
#include "Voronoi.h"
#include "Graphics.h"
#include "Robot.h"

#define M_PI   3.14159265358979323846  /*pi*/

//Approccio seguito: possibilit� di spostare a piacimento i lati dell'AreaBox quadrata rispetto all'origine fisso del sistema di riferimento.
//L'origine (0.0,0.0) rispetto al quale sono espresse le coordinate � fisso, si sposta solo l'AreaBox agendo sui relativi parametri
//-AreaBox parametrizzata in base a parametri AREA_LEFT, AREA_BOTTOM variabili a piacere
//-set(view) parametrizzato per portarsi automaticamente sulla box (gestione grafica)
//-le coordinate dei punti sono univoche per tutto il piano di lavoro --> per poter contribuire alla generazione del diagramma � necessario che il punto
// abbia coordinate interna alla AreaBox (o equivalentemente che l'AreaBox sia spostata in modo tale da avere quel punto al suo interno).

//Diagram/Box parameters
constexpr double AREA_SIZE_x = 20.0;             //Lato della box globale lungo x
constexpr double AREA_SIZE_y = 20.0;     //Lato della box globale lungo y: se uguale = quadrata

constexpr double ROBOT_RANGE = 3.0;     //Range di percezione singolo robot (= met� lato box locale)

//Lati AreaBox (definiscono la posizione del punto in basso a sinistra dell'AreaBox rispetto all'origine degli assi fisso)
//es. AREA_LEFT = -0.5, AREA_BOTTOM = 0.3, AREA_SIZE = 5.0 ----> VERTICI AreaBox: V1(-0.5,-0.3), V2(4.5,-0.3), V3(4.5,4.7), V4(-0.5,4.7)
//quindi per poter divenire sito e contribuire alla generazione del diagramma il punto deve avere coordinate -0.5<=point.x<=4.5 && -0.3<=point.y<=4.7
constexpr double AREA_LEFT = -10;
constexpr double AREA_BOTTOM = -10;
//prova AREA_LEFT = -5.0, AREA_BOTTOM = 0.0 e controlla gli elementi del vettore "points" per capire (per gaussiana mettere un punto compatibile, es PT_X=-3.0, PT_Y=3.0)

//Calcolo Centroidi (Scelta Modalit�)
const bool not_uniform = true;                     //M=0 => centroidi geometrici; M=1 => gaussiana semplice; M=2 => gaussiana multipla
const bool exploration = false;

//Gaussian Density Function parameters
constexpr double PT_X = 8;           //coordinata x punto di interesse (4)
constexpr double PT_Y = 8;           //coordinata y punto di interesse (4)
constexpr double VAR = 1;           //varianza (dispersione dei siti attorno al punto di interesse) (0.2)

//Multiple Gaussian Density Functions parameters --> multiple points and vars
//1.
constexpr double PT1_X = 3.0;
constexpr double PT1_Y = 1.0;
constexpr double VAR1 = 0.2;
//2.
constexpr double PT2_X = 1.0;
constexpr double PT2_Y = 3.0;
constexpr double VAR2 = 0.2;
//n.
//...

bool centroids_vis_on = false;      //visualizzazione storico centroidi/next centroid
bool enter = true;                 //possibilit� visualizzazione dinamica next centroid
bool variance_circles_on = true;   //visualizzazione cerchi varianza gaussiana

unsigned int i = 0;                 //intero per indicizzazione iterazioni - per debugging


int main()
{
    //Debug
    std::cout << "Iteration #" << i << std::endl;

    //--------------------------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------Definizioni Iniziali-------------------------------------------------------------------------
    //Definizione vettore di tutti i punti in area di lavoro

    // std::vector<Vector2<double>> points = {{0.12,0.4}, {0.9,0.1}, {0.35, 0.12}, {0.45,0.45}, {0.25,0.75}, {1.05,1.15}, {2.37,1.45}, {-2.5,2.5}, {-2.0,1.5}, {-3.5,1.0}};
    // std::vector<Robot<double>> Robots = {{0.12,0.4}, {0.9,0.1}, {0.35, 0.12}, {0.45,0.45}, {0.25,0.75}, {1.05,1.15}, {2.37,1.45}, {-2.5,2.5}, {-2.0,1.5}, {-3.5,1.0}};
    std::vector<Vector2<double>> points = {{1,1}, {1,-0.5}, {-2,1.5}, {-4.0,-1.0}, {-0.5,-6.0}};
    std::vector<Robot<double>> Robots = {{1,1}, {1,-2}, {-2,1.5}, {-4.0,-1.0}, {-0.5,-6.0}};

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x+AREA_LEFT, AREA_SIZE_y+AREA_BOTTOM};  //parametrizzato (OK)
    //Definizione range del singolo robot (Box locale)
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};
    //Box<double> HalfRangeBox{-ROBOT_RANGE/2, -ROBOT_RANGE/2, ROBOT_RANGE/2, ROBOT_RANGE/2};
    // Box<double> ObstacleBox{OBSTACLE_LEFT,OBSTACLE_BOTTOM,OBSTACLE_SIZE_x+OBSTACLE_LEFT,OBSTACLE_SIZE_y+OBSTACLE_BOTTOM};

    //Definizione punti di interesse centroidi gaussiani e relative varianze
    std::vector<double> VARs = {VAR};
    std::vector<Vector2<double>> MEANs = {{PT_X,PT_Y}};
    //std::vector<double> VARs = {VAR1, VAR2};
    //std::vector<Vector2<double>> MEANs = {{PT1_X, PT1_Y}, {PT2_X, PT2_Y}};
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------------------------------
    //Debug e Correzioni iniziali-----------------------------------------------------------------------------------------------------------------
    std::cout << "Coordinate globali punti: " << std::endl;
    for(std::size_t i=0; i<points.size(); ++i){
        std::cout << points[i] << std::endl;
    }
    std::cout << "-------------------------------------\n";
    //Esclusione punti esterni all'area di lavoro
    points = adjustPointsVector(points, AreaBox);
    //Debug
    std::cout << "Coordinate globali punti interni: " << std::endl;
    for(std::size_t i=0; i<points.size(); ++i){
        std::cout << points[i] << std::endl;
    }
    std::cout << "-------------------------------------\n";

    //--------------------------------------------------------------------------------------------------------------------------------------------
    //Generazione Diagrammi-----------------------------------------------------------------------------------------------------------------------
    //Generazione diagramma centralizzato globale
    auto diagram = generateCentralizedDiagram(points, AreaBox);

    //Calcolo vettore dei diagrammi decentralizzati
    // RangeBox = adjustObstacleRangeBox(RangeBox,points[0],ROBOT_RANGE,AreaBox,ObstacleBox);
    auto diagrams = generateDecentralizedDiagrams(points, RangeBox, ROBOT_RANGE, AreaBox);
    //Debug
    //auto C = computePolygonCentroid(diagrams[0], Vector2<double> {PT_X, PT_Y}, VAR);
    //auto C = computePolygonCentroid(diagrams[0], MEANs, VARs);
    //std::cout << "===============================================TEMP=== : "<<C<<"\n"<<std::scientific;
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------------------------------
    //Gestione Grafica----------------------------------------------------------------------------------------------------------------------------
    Graphics graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, VAR};

    //Inizializzazione vettore che conterr� tutti i punti per la visualizzazione dello storico dei centroidi e intero per numero punti diagramma
    unsigned int NbPoints = points.size();                      //numero di punti interni all'area
    auto centroids = std::vector<Vector2<double>>();            //inizializzazione centroidi
    auto prev_global_points = std::vector<Vector2<double>>();   //posizioni globali precedenti (vettore cresce iterativamente di dimensione)
    auto local_centroids = std::vector<Vector2<double>>();      //prossimi centroidi locali
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    //Spin-----------------------------------------------------------------------------------------------------------------------------------
    while(graphics.isOpen()){
        //Create event
        sf::Event event;
        //Event loop
        while(graphics.window->pollEvent(event)){
            //Identifying event type
            switch(event.type){
                //Closing window event
                case sf::Event::Closed:
                    graphics.close();
                    break;
                //Evento pressione tasto
                case sf::Event::KeyReleased:
                    if(event.key.code == sf::Keyboard::C){
                        //Salvo ad ogni iterazione i punti precedenti nel vettore (serve per visualizzazione grafica, non � essenziali ai fini del funzionamento)
                        for(const auto& point : points){
                            prev_global_points.push_back(point);
                        }

                        enter = false; //per visualizzazione centroide successivo
                        ++i;
                        std::cout << "\n\n\n\nIteration #" << i << std::endl;
                        if(!not_uniform){
                            centroids = computeCentroids(diagrams);    //Centroidi diposizione uniforme (OK)
                        }
                        else {
                            //centroids = computeGaussianCentroids(diagrams, p_t, VAR);   //centroidi gaussiani (OK)
                            centroids = computeDiagramsCentroids(diagrams, MEANs, VARs);
                        }

                        std::cout << "-----------------------------------------------------\n";
                        std::cout << "Centroide locale/Spostamento da posizione centrale: \n";
                        for(std::size_t i=0; i<centroids.size(); ++i){
                            std::cout << centroids[i] << std::endl;
                        }

                        points = updatepoints(diagrams, centroids);     //Aggiornamento posizioni globali
                        std::cout << "-----------------------------------------------------\n";
                        std::cout << "Vettore points aggiornato: \n";
                        for(std::size_t i=0; i<points.size(); ++i){
                            std::cout << points[i] << std::endl;
                        }
                        diagrams = generateDecentralizedDiagrams(points, RangeBox, ROBOT_RANGE, AreaBox);  //Ricalcolo diagrammi decentralizzati

                        if(!graphics.decentralized_on){
                            diagram = generateCentralizedDiagram(points, AreaBox);   //Rigenerazione diagramma centralizzato
                        }
                        else{
                            graphics.decentralized_on = true;
                            switch(graphics.num){
                                case 0:
                                    diagram = diagrams[0];
                                    break;
                                case 1:
                                    diagram = diagrams[1];
                                    break;
                                case 2:
                                    diagram = diagrams[2];
                                    break;
                                case 3:
                                    diagram = diagrams[3];
                                    break;
                                case 4:
                                    diagram = diagrams[4];
                                    break;
                                case 5:
                                    diagram = diagrams[5];
                                    break;
                                case 6:
                                    diagram = diagrams[6];
                                    break;
                            }
                        }
                        //Visualizzazione centroide successivo
                        if(centroids_vis_on){
                            if(!not_uniform){
                                local_centroids = computeCentroids(diagrams);
                            }
                            else {
                                //local_centroids = computeGaussianCentroids(diagrams, p_t, VAR);
                                local_centroids = computeDiagramsCentroids(diagrams, MEANs, VARs);
                            }
                        }
                    }
                    else if(event.key.code == sf::Keyboard::R){
                        graphics.decentralized_on = false;
                        diagram = generateCentralizedDiagram(points, AreaBox);
                    }
                    else if(event.key.code == sf::Keyboard::V){
                        centroids_vis_on = !centroids_vis_on;
                        if(centroids_vis_on && !enter){
                            if(!not_uniform){
                                local_centroids = computeCentroids(diagrams);
                            }
                            else {
                                //local_centroids = computeGaussianCentroids(diagrams, p_t, VAR);
                                local_centroids = computeDiagramsCentroids(diagrams, MEANs, VARs);
                            }
                            enter = true;
                        }
                    }
                    else if(event.key.code == sf::Keyboard::G){
                        variance_circles_on = !variance_circles_on;
                    }
                    else if(event.key.code == sf::Keyboard::Num0){
                        graphics.decentralized_on = true;
                        graphics.num = 0;
                        diagram = diagrams[0];
                    }
                    else if(event.key.code == sf::Keyboard::Num1){
                        graphics.decentralized_on = true;
                        graphics.num = 1;
                        diagram = diagrams[1];
                    }
                    else if(event.key.code == sf::Keyboard::Num2){
                        graphics.decentralized_on = true;
                        graphics.num = 2;
                        diagram = diagrams[2];
                    }
                    else if(event.key.code == sf::Keyboard::Num3){
                        graphics.decentralized_on = true;
                        graphics.num = 3;
                        diagram = diagrams[3];

                    }
                    else if(event.key.code == sf::Keyboard::Num4){
                        graphics.decentralized_on = true;
                        graphics.num = 4;
                        diagram = diagrams[4];
                    }
                    else if(event.key.code == sf::Keyboard::Num5){
                        graphics.decentralized_on = true;
                        graphics.num = 5;
                        diagram = diagrams[5];
                    }
                    else if(event.key.code == sf::Keyboard::Num6){
                        graphics.decentralized_on = true;
                        graphics.num = 6;
                        diagram = diagrams[6];
                    }
                    break;
                //Rotella mouse per zoom in e zoom out
                case sf::Event::MouseWheelScrolled:
                    if(event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel){
                        if(event.mouseWheelScroll.delta == -1){
                            graphics.view->zoom(1.25);
                            graphics.window->setView(*graphics.view);
                        }
                        else{
                            graphics.view->zoom(0.8);
                            graphics.window->setView(*graphics.view);
                        }
                    }
                    break;
                //Other events
                default:
                    break;
            }
        }
        //Initialize window
        graphics.clear();
        //Visualizzazione Diagramma di Voronoi
        graphics.drawDiagram(diagram, diagrams);
        graphics.drawPoints(diagram, diagrams);

        //Visualizzazione Storico Centroidi (diagramma globale) e centroide successivo (diagramma decentralizzato)
        if(centroids_vis_on){
            if(!graphics.decentralized_on){
                graphics.drawHistory(prev_global_points, NbPoints);
            }
            else{
                graphics.drawNextCentroid(local_centroids, diagrams);
            }
        }
        //Visualizzazione Contour Gaussiani
        if(not_uniform && variance_circles_on){
            graphics.drawGaussianContours(MEANs, VARs);
        }
        //Visualizzazione sistema di riferimento
        graphics.drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));

        //Display window
        graphics.display();
    }
}