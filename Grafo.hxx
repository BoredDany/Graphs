//
// Created by Dany on 7/11/2023.
//

#include "Grafo.h"
#include <vector>
#include <list>
#include <utility>

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
void Graph<T, C>::addVertex(T& vertex){

}

template < class T, class C >
void Graph<T, C>::addEdge(T& origin, T& destination, C cost){

}

//----------------------------------------------------------------------------------------------

//searching

template < class T, class C >
T Graph<T, C>::searchVertice(T& vertex){
    return nullptr;
}

template < class T, class C >
std::pair < int, C > Graph<T, C>::searchEdge(T& origin, T& destination){
    return nullptr;
}

//----------------------------------------------------------------------------------------------

//deleting

template < class T, class C >
void Graph<T, C>::deleteVertex(T& vertex){

}

template < class T, class C >
void Graph<T, C>::deleteEdge(T& origin, T& destination){

}

//----------------------------------------------------------------------------------------------

//general info

template < class T, class C >
int Graph<T, C>::numVertices(){
    return -1;
}

template < class T, class C >
int Graph<T, C>::numEdges(){
    return -1;
}

//----------------------------------------------------------------------------------------------

//tours

template < class T, class C >
void Graph<T, C>::plain(){

}

template < class T, class C >
void Graph<T, C>::bfs(){

}

template < class T, class C >
void Graph<T, C>::dfs(){

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