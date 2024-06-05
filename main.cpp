#include<bits/stdc++.h>
using namespace std;
#include "util/graph.h"
#include "util/path.h"
#include "util/agorithm.h"
#include "util/initPopulation.h"
int main(int argc, char * argv[]){

    clock_t startt = clock();
    inputGraph(argv[0], 16);
    freopen("D:\\Learning\\.vscode\\ttth\\out.txt", "w", stdout);
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
        cout << endl << "MOPSOES angle: " << gPath->angle << " \t,distance: " << gPath->distance << endl;
        pathFunc(astarRes);
        
    // changeToInitGraph();
    clock_t endd = clock();
    cout << endl << endd - startt;
}