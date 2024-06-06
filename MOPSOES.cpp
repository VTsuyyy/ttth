#include<bits/stdc++.h>
using namespace std;
#include "util/graph.h"
#include "util/path.h"
#include "util/agorithm.h"
#include "util/initPopulation.h"
int main(int argc, char * argv[]){
    srand(time(0));
    clock_t startt = clock();
    inputGraph(argv[0], 15);
    // freopen("output\\out2.txt", "w", stdout);
    // changeToNewGraph()

    markObstacle();
    
    for(int i=0; i<30; i++){
        // findByES(start, finish);
        point *tmp = new point(start->x, start->y, nullptr), *tmp1 = new point(finish->x, finish->y, nullptr);
        normalDirect[i] = initRandPath(tmp, tmp1);
        population[i] = new path(tmp);
        // normalDirect[i] = (rand() % 100) & 1;
        numPopulations=i+1;
    }

    // probabilisticMap();
    aStar();
    gPath = astarRes;
    // setPathFromAstar();
    // resetGraphStatus();
    // cout << -1 << endl;
    
    PSOES(60.0, 10000);
    lastSocial();
    // cout << pathLen <<endl;
    for(int i = 0; i < numPopulations; i++){
        if(i != 0) cout << -1 << endl;
        point *tmp = population[i]->begin;
        while(tmp != nullptr) {
            cout << tmp->x/2 << " " << tmp->y/2 << endl;
            tmp = tmp->next;
        }
    }
    cout << -2 << endl;
        cout << endl << "MOPSOES angle: " << gPath->angle << " \t,distance: " << gPath->distance << endl;
        pathFunc(astarRes);
        
    // changeToInitGraph();
    clock_t endd = clock();
    cout << endl << endd - startt;
}