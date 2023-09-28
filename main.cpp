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

// Approach: The ability to freely move the sides of the square AreaBox relative to the fixed origin of the reference system.
// The origin (0.0, 0.0) with respect to which the coordinates are expressed is fixed; only the AreaBox is moved by manipulating its parameters.
// - The AreaBox is parameterized based on the variables AREA_LEFT and AREA_BOTTOM.
// - The set(view) function is parameterized to automatically position itself on the box (graphics management).
// - The coordinates of the points are unique throughout the workspace. To contribute to the diagram generation,
//   the point must have coordinates within the AreaBox or the AreaBox must be moved so that the point is inside it.

// Diagram/Box parameters
constexpr double AREA_SIZE_x = 20.0;             // Side of the global box along x
constexpr double AREA_SIZE_y = 20.0;             // Side of the global box along y; if equal, it's a square

constexpr double ROBOT_RANGE = 3.0;              // Single robot perception range (= half of local box side)

// Sides of the AreaBox (define the position of the bottom-left point of the AreaBox relative to the fixed axis origin)
// e.g., AREA_LEFT = -0.5, AREA_BOTTOM = 0.3, AREA_SIZE = 5.0 ---> AreaBox Vertices: V1(-0.5, -0.3), V2(4.5, -0.3), V3(4.5, 4.7), V4(-0.5, 4.7)
// So, to become a site and contribute to the diagram generation, the point must have coordinates -0.5 <= point.x <= 4.5 && -0.3 <= point.y <= 4.7
constexpr double AREA_LEFT = -10;
constexpr double AREA_BOTTOM = -10;
// For example, try AREA_LEFT = -5.0, AREA_BOTTOM = 0.0 and check the elements of the "points" vector to understand (for Gaussian, place a compatible point, e.g., PT_X = -3.0, PT_Y = 3.0)

// Centroid Calculation (Mode Selection)
const bool not_uniform = true;                   // M=0 => geometric centroids; M=1 => simple Gaussian; M=2 => multiple Gaussians
const bool exploration = false;

// Gaussian Density Function parameters
constexpr double PT_X = 8;           // x-coordinate of the point of interest (4)
constexpr double PT_Y = 8;           // y-coordinate of the point of interest (4)
constexpr double VAR = 1;            // Variance (dispersion of sites around the point of interest) (0.2)

// Multiple Gaussian Density Functions parameters --> multiple points and variances
// 1.
constexpr double PT1_X = 3.0;
constexpr double PT1_Y = 1.0;
constexpr double VAR1 = 0.2;
// 2.
constexpr double PT2_X = 1.0;
constexpr double PT2_Y = 3.0;
constexpr double VAR2 = 0.2;
// n.
// ...

bool centroids_vis_on = false;      // Visualization of centroid history/next centroid
bool enter = true;                 // Dynamic visualization of next centroid
bool variance_circles_on = true;   // Visualization of Gaussian variance circles

unsigned int i = 0;                // Integer for iteration indexing - for debugging

int main()
{
    //Debug
    std::cout << "Iteration #" << i << std::endl;

    //--------------------------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------Initial Definitions-------------------------------------------------------------------------
    // Definition of the vector containing all the points within the workspace

    // std::vector<Vector2<double>> points = {{0.12,0.4}, {0.9,0.1}, {0.35, 0.12}, {0.45,0.45}, {0.25,0.75}, {1.05,1.15}, {2.37,1.45}, {-2.5,2.5}, {-2.0,1.5}, {-3.5,1.0}};
    // std::vector<Robot<double>> Robots = {{0.12,0.4}, {0.9,0.1}, {0.35, 0.12}, {0.45,0.45}, {0.25,0.75}, {1.05,1.15}, {2.37,1.45}, {-2.5,2.5}, {-2.0,1.5}, {-3.5,1.0}};
    std::vector<Vector2<double>> points = {{1,1}, {1,-0.5}, {-2,1.5}, {-4.0,-1.0}, {-0.5,-6.0}};
    std::vector<Robot<double>> Robots = {{1,1}, {1,-2}, {-2,1.5}, {-4.0,-1.0}, {-0.5,-6.0}};

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x+AREA_LEFT, AREA_SIZE_y+AREA_BOTTOM};  // Parameterized (OK)
    // Definition of the range for each individual robot (Local Box)
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};
    // Box<double> HalfRangeBox{-ROBOT_RANGE/2, -ROBOT_RANGE/2, ROBOT_RANGE/2, ROBOT_RANGE/2};
    // Box<double> ObstacleBox{OBSTACLE_LEFT,OBSTACLE_BOTTOM,OBSTACLE_SIZE_x+OBSTACLE_LEFT,OBSTACLE_SIZE_y+OBSTACLE_BOTTOM};

    // Definition of Gaussian centroid points and their variances
    std::vector<double> VARs = {VAR};
    std::vector<Vector2<double>> MEANs = {{PT_X, PT_Y}};
    //std::vector<double> VARs = {VAR1, VAR2};
    //std::vector<Vector2<double>> MEANs = {{PT1_X, PT1_Y}, {PT2_X, PT2_Y}};
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------------------------------
    // Debugging and Initial Corrections-----------------------------------------------------------------------------------------------------------------
    std::cout << "Global coordinates of points: " << std::endl;
    for(std::size_t i=0; i<points.size(); ++i){
        std::cout << points[i] << std::endl;
    }
    std::cout << "-------------------------------------\n";
    // Exclusion of points outside the workspace
    points = adjustPointsVector(points, AreaBox);
    // Debug
    std::cout << "Global coordinates of internal points: " << std::endl;
    for(std::size_t i=0; i<points.size(); ++i){
        std::cout << points[i] << std::endl;
    }
    std::cout << "-------------------------------------\n";
    //--------------------------------------------------------------------------------------------------------------------------------------------
    // Diagram Generation-----------------------------------------------------------------------------------------------------------------------
    // Generation of the centralized global diagram
    auto diagram = generateCentralizedDiagram(points, AreaBox);

    // Calculation of the vector of decentralized diagrams
    // RangeBox = adjustObstacleRangeBox(RangeBox,points[0],ROBOT_RANGE,AreaBox,ObstacleBox);
    auto diagrams = generateDecentralizedDiagrams(points, RangeBox, ROBOT_RANGE, AreaBox);
    // Debug
    //auto C = computePolygonCentroid(diagrams[0], Vector2<double> {PT_X, PT_Y}, VAR);
    //auto C = computePolygonCentroid(diagrams[0], MEANs, VARs);
    //std::cout << "===============================================TEMP=== : "<<C<<"\n"<<std::scientific;
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------------------------------------
    // Graphics Handling----------------------------------------------------------------------------------------------------------------------------
    Graphics graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, VAR};

    // Initialization of the vector that will contain all points for displaying the centroid history and an integer for the number of diagram points
    unsigned int NbPoints = points.size();                      // Number of points within the area
    auto centroids = std::vector<Vector2<double>>();            // Initialization of centroids
    auto prev_global_points = std::vector<Vector2<double>>();   // Previous global positions (vector grows iteratively in size)
    auto local_centroids = std::vector<Vector2<double>>();      // Next local centroids
    //--------------------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------------------------------------------

    // Spin-----------------------------------------------------------------------------------------------------------------------------------
    while(graphics.isOpen()){
        // Create event
        sf::Event event;
        // Event loop
        while(graphics.window->pollEvent(event)){
            // Identify event type
            switch(event.type){
                // Closing window event
                case sf::Event::Closed:
                    graphics.close();
                    break;
                // Key press event
                case sf::Event::KeyReleased:
                    if(event.key.code == sf::Keyboard::C){
                        // Save the previous points at each iteration (used for graphic visualization, not essential for operation)
                        for(const auto& point : points){
                            prev_global_points.push_back(point);
                        }

                        enter = false; // for visualizing the next centroid
                        ++i;
                        std::cout << "\n\n\n\nIteration #" << i << std::endl;
                        if(!not_uniform){
                            centroids = computeCentroids(diagrams);    // Centroids for uniform distribution (OK)
                        }
                        else {
                            //centroids = computeGaussianCentroids(diagrams, p_t, VAR);   // Gaussian centroids (OK)
                            centroids = computeDiagramsCentroids(diagrams, MEANs, VARs);
                        }

                        std::cout << "-----------------------------------------------------\n";
                        std::cout << "Local Centroid/Move from Central Position: \n";
                        for(std::size_t i=0; i<centroids.size(); ++i){
                            std::cout << centroids[i] << std::endl;
                        }

                        points = updatepoints(diagrams, centroids);     // Update global positions
                        std::cout << "-----------------------------------------------------\n";
                        std::cout << "Updated points vector: \n";
                        for(std::size_t i=0; i<points.size(); ++i){
                            std::cout << points[i] << std::endl;
                        }
                        diagrams = generateDecentralizedDiagrams(points, RangeBox, ROBOT_RANGE, AreaBox);  // Recalculate decentralized diagrams

                        if(!graphics.decentralized_on){
                            diagram = generateCentralizedDiagram(points, AreaBox);   // Regenerate centralized diagram
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
                        // Visualize the next centroid
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
                // Mouse wheel for zooming in and out
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
                // Other events
                default:
                    break;
            }
        }
        // Initialize the window
        graphics.clear();
        // Display the Voronoi Diagram
        graphics.drawDiagram(diagram, diagrams);
        graphics.drawPoints(diagram, diagrams);

        // Display Centroid History (Global Diagram) and Next Centroid (Decentralized Diagram)
        if(centroids_vis_on){
            if(!graphics.decentralized_on){
                graphics.drawHistory(prev_global_points, NbPoints);
            }
            else{
                graphics.drawNextCentroid(local_centroids, diagrams);
            }
        }
        // Display Gaussian Contours
        if(not_uniform && variance_circles_on){
            graphics.drawGaussianContours(MEANs, VARs);
        }
        // Display the reference system
        graphics.drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));

        // Display the window
        graphics.display();
    }
}