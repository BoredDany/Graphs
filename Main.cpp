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
    int origen, destino, vertice;

    readVertices("c.txt", g);
    readEdges("c.txt", g);
    cout<<"VERTICES: "<<endl;
    g.plain();
    cout<<"\nEDGES: "<<endl;
    g.showEdges();
    cout<<"\nNUM VERTICES: ";
    cout<<g.numVertices();
    cout<<"\nNUM EDGES: ";
    cout<<g.numEdges();

    /*cout << "ELIMINAR ARISTA" << endl;
    cout << "Origen: ";
    cin >> origen;
    cout << "Destino: ";
    cin >> destino;
    g.deleteEdge(origen, destino);
    cout<<"\nEDGES: "<<endl;
    g.showEdges();*/

    cout << "ELIMINAR VERTICE" << endl;
    cout << "Vertice: ";
    cin >> vertice;
    g.deleteVertex(vertice);
    cout<<"VERTICES: "<<endl;
    g.plain();
    cout<<"\nEDGES: "<<endl;
    g.showEdges();

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
                graph.addEdge(pais, vecino, 0);
            }
        }
    }else{
        std::cout<<"Archivo de conexiones no leido"<<std::endl;
    }
    file.close();
}