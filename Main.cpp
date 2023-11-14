//
// Created by Dany on 7/11/2023.
//
#include "Grafo.h"
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <utility>
#include <sstream>
#include <fstream>
#include <cstring>
using namespace std;

void readVertices(string archivo, Graph<int, int>& graph);
void readEdges(string archivo, Graph<int, int>& graph);

int main(){
    Graph <int, int> g;
    int origen, destino, vertice, v, e, newV, ori, dest, cost;

    //readVertices("c.txt", g);
    //readEdges("c.txt", g);
    cout<<"vertices:";
    cin>>v;
    for(int i = 0 ; i < v ; i++){
        cout<<"vertex:";
        cin>>newV;
        g.addVertex(newV);
    }

    for(int i = 0 ; i < g.numVertices() ; i++){
        cout<<"vertex:"<< g.getVertices()[i]<<endl;
        cout<<"aristas:";
        cin>>e;
        for(int j = 0 ; j < e ; j++){
            cout<<"destino:";
            cin>>dest;
            cout<<"costo:";
            cin>>cost;
            g.addEdge(g.getVertices()[i], dest, cost);
        }
    }

    cout<<"VERTICES: "<<endl;
    g.plain();
    cout<<"\nEDGES: "<<endl;
    g.showEdges();
    /*cout<<"\nNUM VERTICES: ";
    cout<<g.numVertices();
    cout<<"\nNUM EDGES: ";
    cout<<g.numEdges();

    cout << "ELIMINAR ARISTA" << endl;
    cout << "Origen: ";
    cin >> origen;
    cout << "Destino: ";
    cin >> destino;
    g.deleteEdge(origen, destino);
    cout<<"\nEDGES: "<<endl;
    g.showEdges();

    cout << "ELIMINAR VERTICE" << endl;
    cout << "Vertice: ";
    cin >> vertice;
    g.deleteVertex(vertice);
    cout<<"VERTICES: "<<endl;
    g.plain();
    cout<<"\nEDGES: "<<endl;
    g.showEdges();

    cout<<"BFS: "<<endl;
    g.bfs();

    cout<<"\nDFS: "<<endl;
    g.dfs();
    bool connected = g.connected();
    cout << "Conectado: " << connected << endl;
    g.connectedComponents();
    vertice = 1;
    g.prim(vertice);
    g.dijkstra(vertice);
    g.bridgeEdges();*/

    vertice = 1;
    g.prim(vertice);
    g.dijkstra(vertice);

    return 0;
}

void readVertices(string archivo, Graph<int, int>& graph){
    std::ifstream file (archivo);
    std::string line, word;

    if(file.is_open()){
        while(getline(file,line,'\n')){
            std::stringstream ss(line);
            getline(ss,word,'-');
            int pais = stoi(word);
            graph.addVertex(pais);
        }
    }else{
        std::cout<<"Archivo de conexiones no leido"<<std::endl;
    }
    file.close();
}

void readEdges(string archivo, Graph<int, int>& graph){
    std::ifstream file (archivo);
    std::string line, word;
    int costo = 0;

    if(file.is_open()){
        while(getline(file,line,'\n')){
            std::stringstream ss(line);
            getline(ss,word,'-');
            int pais = stoi(word);
            //cout<< "\npais " << pais <<": ";
            while(getline(ss,word,';')){
                int vecino = stoi(word);
                //cout<< vecino << " ";
                graph.addEdge(pais, vecino, costo);
                costo++;
            }
        }
    }else{
        std::cout<<"Archivo de conexiones no leido"<<std::endl;
    }
    file.close();
}