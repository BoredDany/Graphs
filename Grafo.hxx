//
// Created by Dany on 7/11/2023.
//

#include "Grafo.h"
#include <vector>
#include <list>
#include <utility>
#include <iostream>
#include <queue>
#include <climits>
#include <limits>

//constructors

template <class T, class C>
Graph<T, C>::Graph() {

}

//----------------------------------------------------------------------------------------------

//getters

template < class T, class C >
std::vector < T > Graph<T, C>::getVertices(){
    return this->vertices;
}

template < class T, class C >
std::vector < std::list < std::pair < int, C > > > Graph<T, C>::getEdges(){
    return this->edges;
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
            break;
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

template <class T, class C>
bool Graph<T, C>::deleteVertex(T& vertex) {
    int vertexIndex = searchVertice(vertex);
    int ind = 0;
    typename std::vector<T>::iterator itEVertex;
    typename std::vector<std::list<std::pair<int, C>>>::iterator itEdges, itEEdge;
    typename std::list<std::pair<int, C>>::iterator itList, itEList;

    if (vertexIndex != -1) {
        ind = 0;
        for (itEdges = this->edges.begin(); itEdges != this->edges.end(); itEdges++, ind++) {
            if (ind == vertexIndex) {
                itEEdge = itEdges; // guarda posicion a eliminar del vector de aristas
            } else {
                for (itList = itEdges->begin(); itList != itEdges->end(); itList++) {
                    if (itList->first == vertexIndex) {
                        itEList = itList;
                        itEdges->erase(itEList);
                        break;
                    }
                }
            }
        }
        if (itEEdge != this->edges.end()) {
            this->edges.erase(itEEdge);
        }
        itEVertex = this->vertices.begin() + vertexIndex;
        this->vertices.erase(itEVertex);
        updateEdges(vertexIndex);
        return true;
    }
    return false;
}

template < class T, class C >
void Graph<T, C>::updateEdges(int index){
    typename  std::list < std::pair < int, C > >::iterator itL;

    for(int i = 0 ; i < this->edges.size() ; i++){
        for(itL = this->edges[i].begin() ; itL != this->edges[i].end() ; itL++){
            if((*itL).first > index){
                (*itL).first-=1;
            }
        }
    }
}

template < class T, class C >
bool Graph<T, C>::deleteEdge(T& origin, T& destination){
    int i_ori = searchVertice(origin);
    int i_des = searchVertice(destination);
    typename  std::list < std::pair < int, C > >::iterator itList, posE;

    if ( i_ori != -1 && i_des != -1 ) {
        for (itList = this->edges[i_ori].begin(); itList != this->edges[i_ori].end(); itList++) {
            if (itList->first == i_des){
                posE = itList;
                break;
            }
        }
        this->edges[i_ori].erase(posE);
        return true;
    }
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
    std::vector<bool> visited(this->vertices.size(), false);
    std::queue<int> vertexQueue;

    for (int i = 0; i < this->vertices.size(); i++) {
        if (!visited[i]) {
            doBFS(i, visited, vertexQueue);
        }
    }
}

template < class T, class C >
void Graph<T, C>::doBFS(int startVertex, std::vector<bool>& visited, std::queue<int>& vertexQueue){
    visited[startVertex] = true;
    vertexQueue.push(startVertex);

    while (!vertexQueue.empty()) {
        int currentVertex = vertexQueue.front();
        vertexQueue.pop();

        std::cout << this->vertices[currentVertex] << ", ";

        typename  std::list < std::pair < int, C > >::iterator itL;
        itL = this->edges[currentVertex].begin();

        for( ; itL != this->edges[currentVertex].end() ; itL++){
            int neighbor = (*itL).first;
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                vertexQueue.push(neighbor);
            }
        }
    }
}

template < class T, class C >
void Graph<T, C>::dfs(){
    std::vector<bool> visited(this->vertices.size(), false);

    for (int i = 0; i < this->vertices.size(); ++i) {
        if (!visited[i]) {
            doDFS(i, visited);
        }
    }
}

template <class T, class C>
void Graph<T, C>::doDFS(int currentVertex, std::vector<bool>& visited) {
    visited[currentVertex] = true;
    std::cout << this->vertices[currentVertex] << ", ";

    typename  std::list < std::pair < int, C > >::iterator itL;
    itL = this->edges[currentVertex].begin();

    for ( ; itL != this->edges[currentVertex].end(); itL++) {
        int neighbor = (*itL).first;
        if (!visited[neighbor]) {
            doDFS(neighbor, visited);
        }
    }
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
    std::list < std::pair < int, int > > bridges;

    for(int i = 0 ; i < this->edges.size() ; i++){
        std::list < std::pair < int, C > > auxEdges = this->edges[i];
        for(typename  std::list < std::pair < int, C > >::iterator itL = auxEdges.begin() ; itL != auxEdges.end() ; itL++){
            this->deleteEdge(this->vertices[i], this->vertices[(*itL).first]);
            if(!this->connected()){
                std::pair < int, int > bridge (i, (*itL).first);
                bridges.push_back(bridge);
            }
            this->addEdge(this->vertices[i], this->vertices[(*itL).first], (*itL).second);
        }
    }

    if(!bridges.empty()){
        for(typename  std::list < std::pair < int, C > >::iterator itB = bridges.begin() ; itB != bridges.end() ; itB++){
            std::cout << "Bridge: " << this->vertices[(*itB).first] << "-" << this->vertices[(*itB).second] << std::endl;
        }
    }
}

template < class T, class C >
void Graph<T, C>::connectedComponents(){
    std::vector<bool> visited(this->vertices.size(), false);
    std::list < std::list < T > > components;
    for (int i = 0; i < this->vertices.size(); ++i) {
        if (!visited[i]) {
            std::list < T >  component;
            getComponents(i, visited, component);
            components.push_back(component);
        }
    }
    for(std::list < T > l: components){
        std::cout << "Component" << std::endl;
        for(T c: l){
            std::cout << c << " ";
        }
        std::cout << std::endl;
    }
}

template < class T, class C >
void Graph<T, C>::getComponents(int currentVertex, std::vector<bool>& visited, std::list < T >& components){
    visited[currentVertex] = true;
    components.push_back(this->vertices[currentVertex]);
    typename  std::list < std::pair < int, C > >::iterator itL;
    itL = this->edges[currentVertex].begin();

    for ( ; itL != this->edges[currentVertex].end(); itL++) {
        int neighbor = (*itL).first;
        if (!visited[neighbor]) {
            getComponents(neighbor, visited, components);
        }
    }
}

template < class T, class C >
bool Graph<T, C>::connected(){
    std::vector<bool> visited(this->vertices.size(), false);
    int numComponents = 0;
    for (int i = 0; i < this->vertices.size(); ++i) {
        if (!visited[i]) {
            doDFSConnected(i, visited);
            numComponents++;
        }
    }
    std::cout << "\nHay " << numComponents << " componentes conectados" << std::endl;
    if(numComponents == 1){
        return true;
    }
    return false;
}

template <class T, class C>
void Graph<T, C>::doDFSConnected(int currentVertex, std::vector<bool>& visited) {
    visited[currentVertex] = true;

    typename  std::list < std::pair < int, C > >::iterator itL;
    itL = this->edges[currentVertex].begin();

    for ( ; itL != this->edges[currentVertex].end(); itL++) {
        int neighbor = (*itL).first;
        if (!visited[neighbor]) {
            doDFSConnected(neighbor, visited);
        }
    }
}

template < class T, class C >
bool Graph<T, C>::stronglyConnected(){
    std::vector<bool> visited(this->vertices.size(), false);

    for(int i = 0 ; i < this->vertices.size() ; i++){
        if(!visited[i]){
            visited[i] = true;
            std::vector <T> desc = getDesc(i);
            std::vector <T> asc = getAsc(i);
            std::set <T> component;
            std::set_intersection(desc.begin(), desc.end(), asc.begin(), asc.end(), std::inserter(component, component.begin()));
            if(component.size() == this->vertices.size()){
                std::cout << "IS STRONGLY CONNECTED" << std::endl;
                return true;
            }
        }
    }
    return false;

}

template <class T, class C>
std::vector <T> Graph<T, C>::getDesc(int i){
    std::vector<bool> visited(this->vertices.size(), false);
    std::vector <T> desc;
    desc.push_back(this->vertices[i]);
    visited[i] = true;
    getDescDFS(i, desc, visited);
    return desc;
}

template <class T, class C>
void Graph<T, C>::getDescDFS(int i, std::vector<T>& desc, std::vector<bool>& visited){
    for(typename std::list < std::pair <int, C> >::iterator it = this->edges[i].begin() ; it != this->edges[i].end() ; it++){
        if(!visited[(*it).first]){
            desc.push_back(this->vertices[(*it).first]);
            visited[(*it).first] = true;
            getDescDFS((*it).first, desc, visited);
        }
    }
}

template <class T, class C>
std::vector <T> Graph<T, C>::getAsc(int i){
    std::vector <T> asc;
    asc.push_back(this->vertices[i]);



    return asc;
}

template <class T, class C>
void Graph<T, C>::prim(T& initial) {
    int n = vertices.size();

    std::vector<bool> visited(n, false);
    std::vector<C> key(n, std::numeric_limits<C>::max());
    std::vector<int> parent(n, -1);

    int initialIndex = searchVertice(initial);

    if (initialIndex == -1) {
        std::cerr << "Error: Initial vertex not found." << std::endl;
        return;
    }

    key[initialIndex] = 0;

    for (int count = 0; count < n - 1; ++count) {
        int u = -1;
        C minKey = std::numeric_limits<C>::max();

        // Find the vertex with the minimum key value
        for (int v = 0; v < n; ++v) {
            if (!visited[v] && key[v] < minKey) {
                minKey = key[v];
                u = v;
            }
        }

        if (u == -1) {
            std::cerr << "Error: Graph is not connected." << std::endl;
            return;
        }

        visited[u] = true;

        // Explore all adjacent vertices of the selected vertex 'u'
        for (typename std::list < std::pair <int, C> >::iterator it = this->edges[u].begin();
                it != this->edges[u].end() ; it++) {
            int v = (*it).first;
            C weight = (*it).second;

            if (!visited[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }

    // Print the minimum spanning tree
    std::cout << "Edges of the Minimum Spanning Tree:" << std::endl;
    for (int i = 1; i < n; ++i) {
        std::cout << "Edge: " << this->vertices[parent[i]] << " - " << this->vertices[i] << " with weight " << key[i] << std::endl;
    }
}

template < class T, class C >
void Graph<T, C>::dijkstra(T& initial) {
    int n = vertices.size();

    std::vector<C> distance(n, std::numeric_limits<C>::max());
    std::vector<int> parent(n, -1);
    std::vector<bool> visited(n, false);

    // Find the index of the start vertex
    int startIndex = searchVertice(initial);

    if (startIndex == -1) {
        std::cerr << "Error: Start vertex not found." << std::endl;
        return;
    }

    distance[startIndex] = 0;

    // Priority queue to store vertices and their distances
    std::priority_queue<std::pair<C, int>, std::vector<std::pair<C, int>>, std::greater<std::pair<C, int>>> pq;
    pq.push({0, startIndex});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) {
            continue; // Skip if the vertex is already visited
        }

        visited[u] = true;

        // Explore all neighbors of the selected vertex 'u'
        for (const auto& neighbor : edges[u]) {
            int v = neighbor.first;
            C edgeCost = neighbor.second;

            if (!visited[v] && distance[u] + edgeCost < distance[v]) {
                distance[v] = distance[u] + edgeCost;
                parent[v] = u;
                pq.push({distance[v], v});
            }
        }
    }

    // Print the edges and distances
    std::cout << "Edges and Distances from vertex " << initial << ":" << std::endl;
    for (int i = 0; i < n; ++i) {
        if (i != startIndex) {
            std::cout << "Edge: " << vertices[parent[i]] << " - " << vertices[i] << " with distance " << distance[i] << std::endl;
        }
    }
}

//----------------------------------------------------------------------------------------------