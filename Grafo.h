//
// Created by Dany on 7/11/2023.
//

#ifndef GRAPHS_GRAFO_H
#define GRAPHS_GRAFO_H

#include <vector>
#include <list>
#include <utility>
#include <queue>

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
    bool addVertex(T& vertex);
    bool addEdge(T& origin, T& destination, C cost);

    //searching
    int searchVertice(T& vertex);
    bool searchEdge(T& origin, T& destination);

    //deleting
    bool deleteVertex(T& vertex);
    bool deleteEdge(T& origin, T& destination);
    void updateEdges(int index);

    //general info
    int numVertices();
    int numEdges();

    //tours
    void plain();
    void bfs();
    void doBFS(int startVertex, std::vector<bool>& visited, std::queue<int>& vertexQueue);
    void dfs();
    void doDFS(int currentVertex, std::vector<bool>& visited);
    void showEdges();

    //algorithms
    void bridgeEdges();

    void connectedComponents();
    void getComponents(int currentVertex, std::vector<bool>& visited, std::list < T >& components);

    bool connected();
    void doDFSConnected(int currentVertex, std::vector<bool>& visited);

    bool stronglyConnected();
    void prim();
    void dijkstra();

};

#include "Grafo.hxx"
#endif //GRAPHS_GRAFO_H
