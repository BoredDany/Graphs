//
// Created by Dany on 7/11/2023.
//

#ifndef GRAPHS_GRAFO_H
#define GRAPHS_GRAFO_H

#include <vector>
#include <list>
#include <utility>

template < class T, class C >
class Graph {

private:
    std::vector < T > vertices;
    std::vector < std::list < std::pair < int, C > > > edges;
public:
    //constructors
    Graph();

    //getters
    std::vector < T > getVertices();
    std::vector < std::list < std::pair < int, C > > > getEdges();

    //setters
    void setVertices(std::vector < T >& vertices);
    void setEdges(std::vector < std::list < std::pair < int, C > > >& edges);

    //inserting
    void addVertex(T& vertex);
    void addEdge(T& origin, T& destination, C cost);

    //searching
    T searchVertice(T& vertex);
    std::pair < int, C > searchEdge(T& origin, T& destination);

    //deleting
    void deleteVertex(T& vertex);
    void deleteEdge(T& origin, T& destination);

    //general info
    int numVertices();
    int numEdges();

    //tours
    void plain();
    void bfs();
    void dfs();

    //algorithms
    void bridgeEdges();
    void connectedComponents();
    bool connected();
    bool stronglyConnected();
    void prim();
    void dijkstra();

};

#include "Grafo.hxx"
#endif //GRAPHS_GRAFO_H
