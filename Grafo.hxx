//
// Created by Dany on 7/11/2023.
//

#include "Grafo.h"
#include <vector>
#include <list>
#include <utility>
#include <iostream>

//constructors

template <class T, class C>
Graph<T, C>::Graph() {

}

//----------------------------------------------------------------------------------------------

//getters

template < class T, class C >
std::vector < T > Graph<T, C>::getVertices(){
    return this.vertices;
}

template < class T, class C >
std::vector < std::list < std::pair < int, C > > > Graph<T, C>::getEdges(){
    return this.edges;
}

//----------------------------------------------------------------------------------------------

//setters

template < class T, class C >
void Graph<T, C>::setVertices(std::vector < T >& vertices){
    this.vertices = vertices;
}

template < class T, class C >
void Graph<T, C>::setEdges(std::vector < std::list < std::pair < int, C > > >& edges){
    this.edges = edges;
}

//----------------------------------------------------------------------------------------------

//inserting

template < class T, class C >
bool Graph<T, C>::addVertex(T& vertex){
    int vertexFound = searchVertice(vertex);
    if(vertexFound == -1){
        this->vertices.push_back(vertex);
        this->edges.push_back(std::list<std::pair<int, C>>());
        return true;
    }
    return false;
}

template < class T, class C >
bool Graph<T, C>::addEdge(T& origin, T& destination, C cost){
    int destinationIndex = searchVertice(destination);
    int originIndex = searchVertice(origin);

    if(!searchEdge(origin, destination) && originIndex != -1 && destinationIndex != -1){
        std::pair < int, C > newEdge (destinationIndex, cost);
        this->edges[originIndex].push_back(newEdge);
        return true;
    }
    return false;
}

//----------------------------------------------------------------------------------------------

//searching

template < class T, class C >
int Graph<T, C>::searchVertice(T& vertex){
    int vertexFound = -1;
    for(int i = 0 ; i < this->vertices.size() ; i++){
        if(this->vertices[i] == vertex){
            vertexFound = i;
        }
    }
    return vertexFound;
}

template < class T, class C >
bool Graph<T, C>::searchEdge(T& origin, T& destination){
    typename  std::list < std::pair < int, C > >::iterator itL;
    int destinationIndex = searchVertice(destination);
    int originIndex = searchVertice(origin);

    if(destinationIndex != -1 && originIndex != -1){
        itL = this->edges[originIndex].begin();
        for(itL = this->edges[originIndex].begin() ; itL != this->edges[originIndex].end() ; itL++){
            if(itL->first == destinationIndex){
                return true;
            }
        }
    }
    return false;
}

//----------------------------------------------------------------------------------------------

//deleting

template < class T, class C >
bool Graph<T, C>::deleteVertex(T& vertex){
    int vertexIndex = searchVertice(vertex);
    if ( vertexIndex != -1) {
        std::vector< std::list< pair<int,C> > >::iterator itA, posE;
        int ind = 0;
        for (itA = aristas.begin(); itA != aristas.end(); itA++, ind++) {
            if (ind == i_vert) {
                posE = itA;
            } else {
                std::list< pair<int,U> >::iterator itList, posEE;
                for (itList = itA->begin(); itList != itA->end(); itList++) {
                    if (itList->first == i_vert) {
                        posEE = itList;
                    }
                }
                itA->erase(posEE);
            }
        }
        aristas.erase(posE);
    }
    return false;
}

template < class T, class C >
bool Graph<T, C>::deleteEdge(T& origin, T& destination){
    return false;
}

//----------------------------------------------------------------------------------------------

//general info

template < class T, class C >
int Graph<T, C>::numVertices(){
    return this->vertices.size();
}

template < class T, class C >
int Graph<T, C>::numEdges(){
    typename  std::list < std::pair < int, C > >::iterator itL;
    int numEdges = 0;

    for(int i = 0 ; i < this->edges.size() ; i++){
        std::list < std::pair < int, C > > auxEdges = this->edges[i];
        itL = auxEdges.begin();
        for(itL = auxEdges.begin() ; itL != auxEdges.end() ; itL++){
            numEdges++;
        }
    }
    return numEdges;
}

//----------------------------------------------------------------------------------------------

//tours

template < class T, class C >
void Graph<T, C>::plain(){
    for(int i = 0 ; i < this->vertices.size() ; i++){
        std::cout << this->vertices[i] << " ; ";
    }
}

template < class T, class C >
void Graph<T, C>::bfs(){

}

template < class T, class C >
void Graph<T, C>::dfs(){

}

template < class T, class C >
void Graph<T, C>::showEdges(){
    typename  std::list < std::pair < int, C > >::iterator itL;

    for(int i = 0 ; i < this->edges.size() ; i++){
        std::cout << "\n" << this->vertices[i] << ": ";
        std::list < std::pair < int, C > > auxEdges = this->edges[i];
        itL = auxEdges.begin();
        for(itL = auxEdges.begin() ; itL != auxEdges.end() ; itL++){
            std::cout << "(" << this->vertices[(*itL).first] << "," << (*itL).second << ") ";
        }
        std::cout << std::endl;
    }
}

//----------------------------------------------------------------------------------------------

//algorithms

template < class T, class C >
void Graph<T, C>::bridgeEdges(){

}

template < class T, class C >
void Graph<T, C>::connectedComponents(){

}

template < class T, class C >
bool Graph<T, C>::connected(){

}

template < class T, class C >
bool Graph<T, C>::stronglyConnected(){

}

template < class T, class C >
void Graph<T, C>::prim(){

}

template < class T, class C >
void Graph<T, C>::dijkstra(){

}

//----------------------------------------------------------------------------------------------