typedef pair<double, pair<int, int>> pairDistance;
double gDistance[1000][1000] = {};
int visited[1000][1000] = {};
path* astarRes;
path* aStar(){
    set<pairDistance> aStar;
    int xStart = (int)start->x, yStart = (int)start->y, xFinish = (int)finish->x, yFinish = (int)finish->y;
    aStar.insert(make_pair(0.0, make_pair(xStart, yStart)));
    while(aStar.empty() == false) {
        pairDistance newPoint = *aStar.begin();
        aStar.erase(aStar.begin());
        int x = newPoint.second.first, y = newPoint.second.second, past = x*1000+y;
        for(int i = 0; i < 8; ++i) {
            int x1 = x + nearPoint[i][0];
            int y1 = y + nearPoint[i][1];
            if(visited[x1][y1] == 0 && checkValidPoint(x1, y1)){
                visited[x1][y1] = past;
                gDistance[x1][y1] = gDistance[x][y] + sqrt(abs(nearPoint[i][0]) + abs(nearPoint[i][1]));
                aStar.insert(make_pair(gDistance[x1][y1] + distanceToFinish(x1, y1), make_pair(x1, y1)));
                if(x1 == xFinish && y1 == yFinish) goto findSolution;
            }
            else if(checkValidPoint(x1, y1)){
                gDistance[x1][y1] = min(gDistance[x1][y1], gDistance[x][y] + sqrt((double) (abs(nearPoint[i][0]) + abs(nearPoint[i][1]))));
            }
        }
    }
    
    findSolution:
    path *res = new path(gDistance[xFinish][yFinish], 0, nullptr);
    point *p = new point((double) xFinish, (double) yFinish, nullptr);
    int stop = 1000;
    while(--stop > 0){
        int tmp = visited[xFinish][yFinish];
        xFinish = tmp / 1000;
        yFinish = tmp % 1000;
        p = new point((double) xFinish, (double) yFinish, p);
        if(xFinish == xStart && yFinish == yStart) break;
    }
    
    res->begin = p;
    astarRes = res;
    return res;
}

path *currPath, *gPath, *pPath[1000], *population[1000];
int numPopulations = 5, populationMax = 100, pathLen;

void decreaseDimension() {
    for(int i = 0; i < numPopulations; ++i){
        point *p = population[i]->begin->next;
        while(p->next != nullptr && p->next->next != nullptr) {
            double eDis = euclideanDistance(p, p->next), angle = angleThreePoint(p, p->next, p->next->next);
            if(eDis < 15 && eDis > 3 && angle < 0.003) p->next = p->next->next;
            p = p->next;
        }
    }
}

path* makeCopyPath(path *p) {
    point *p1 = p->begin, *q1 = new point(p1->x, p1->y, nullptr);
    path *q = new path(q1);
    p1 = p1->next;
    while(p1 != nullptr) {
        q1->next = new point(p1->x, p1->y, nullptr);
        q1 = q1->next;
        p1 = p1->next;
    }
    pathFunc(q);
    return q;
}

void setPathFromAstar(){
    point *p = astarRes->begin;
    for(int i = 0; i <= populationMax; ++i) {
        point *q = p, *tmp = new point();
        population[i] = new path(tmp);
        while(q != nullptr) {
            tmp->next = new point(q->x, q->y, nullptr);
            tmp = tmp->next;
            q = q->next;
        }
        population[i]->begin = population[i]->begin->next;
        pathFunc(population[i]);
        pPath[i] = population[i];
    }
    gPath = pPath[populationMax];
}

void updateBestPath(){
    for(int i = 0; i < numPopulations; i++) {
        if(compareTwoPath(population[i], pPath[i]) > 0) pPath[i] = makeCopyPath(population[i]);
        if(compareTwoPath(population[i], gPath) > 0){ 
            gPath = population[i];
        }
    }
}

double w0PSO = 0.3, w1PSO = 0.2, w2PSO = 0.2, w3PSO = 0.1, v[1000][200][2] = {}, normalDirect[100];
void updateV(int i){
    int j = 1;
    point *p = population[i]->begin->next, *pp = pPath[i]->begin->next;
    while(p->next != nullptr) {
        v[i][j][0] = w0PSO * v[i][j][0] + w1PSO * (pp->x - p->x);
        v[i][j][1] = w0PSO * v[i][j][1] + w1PSO * (pp->y - p->y);
        ++j;
        p = p->next;
        pp = pp->next;
    }
}


void PSOmigrate(){
    for(int i = 0; i < numPopulations; i++) {
        point *p1 = population[i]->begin, *p = p1->next;
        pair<double, double> q;
        while(p != nullptr && p->next != nullptr){
            int x =  (int) p->x, y = (int) p->y;
            p->x = p->x*0.8 + (p->next->x+p1->x)*10/100;
            p->y = p->y*0.8 + (p->next->y+p1->y)*10/100;
            q = gradientPoint(p->x, p->y);
            p->x = p->x * 0.7 + q.first * 0.3;
            p->y = p->y * 0.7 + q.second * 0.3;
            if(onMapSize((int) p->x, (int) p->y) == false) {
                normalDirect[i] = 1 - normalDirect[i];
            };
            if(checkValidPoint(p) == false) 
                if(normalDirect[i]) 
                    normalLine(p1, p, p->next);
                else normalLine(p->next, p, p1);
            
            if(euclideanDistance(p, p1) > 7){
                p1->next = new point((p->x+p1->x)/2, (p->y+p1->y)/2, p1->next);
                p1 = p1->next;
            }
            else if(euclideanDistance(p, p1) < 3 && angleThreePoint(p1, p, p->next) < 0.2){
                p1->next = p->next;
                p = p1->next;
            }
            else{
                p = p->next;
                p1 = p1->next;
            }
        }
    }
}

double wVPSO = 0.1;
void PSO(){
    for(int i = 0; i < numPopulations; i++) {
        updateV(i);
        point *p = population[i]->begin->next;
        pathLen = pathLength(population[i]);
        for(int j = 1; j < pathLen-1; ++j) {
            p->x += v[i][j][0] * wVPSO;
            p->y += v[i][j][1] * wVPSO;
            p = p->next;
        }
    }
}

double est = 1.5;
path* mutation(path *p, double toiu){
    point *tmp = p->begin, *offPoint = new point(tmp->x, tmp->y, nullptr);
    path *offspring = new path(offPoint);
    tmp = tmp->next;
    double keke = rand() % (int)(pathLen*est/4), x1 = (rand() % 200 - 100) / (toiu), y1 = (rand() % 200 - 100) / (toiu), stt = 0, posPoint = rand() % (pathLen+6)-3;
    while(tmp->next != nullptr) {
        double wei = 1 - min(1.0, abs(stt++ - posPoint) / keke);
        offPoint->next = new point(tmp->x + x1 * wei, tmp->y + y1 * wei, nullptr);
        if(checkValidLine(offPoint, offPoint->next) == false) return p;
        offPoint = offPoint->next;
        tmp = tmp->next;
    }
    offPoint->next = new point(tmp->x, tmp->y, nullptr);
    pathFunc(offspring);
    return offspring;
}

path* combination1(path *p, path *q){
    if(q == nullptr) return nullptr;
    point *p1 = p->begin, *q1 = q->begin->next, *tmp1 = new point(p1->x, p1->y, nullptr);
    p1 = p1->next;
    path *tmp = new path(tmp1);
    while(p1->next != nullptr) {
        tmp1->next = new point((p1->x + q1->x)/2, (p1->y + q1->y)/2, nullptr);
        tmp1 = tmp1->next;
        p1 = p1->next;
        q1 = q1->next;
    }
    tmp1->next = new point(p1->x, p1->y, nullptr);
    // tmp->begin = tmp->begin->next;
    pathFunc(tmp);
    return tmp;
}

int chooseCombine = 52465, modCombine = 9383761; 
path* combination2(path *p, path *q){
    if(q == nullptr) return nullptr;
    point *p1 = p->begin, *q1 = q->begin->next, *tmp1 = new point(p1->x, p1->y, nullptr);
    p1 = p1->next;
    path *tmp = new path(tmp1);
    while(p1->next != nullptr) {
        chooseCombine = (chooseCombine * 6) % modCombine;
        if((chooseCombine & 1) == 1) tmp1->next = new point(p1->x, p1->y, nullptr);
        else tmp1->next = new point(q1->x, q1->y, nullptr);
        tmp1 = tmp1->next;
        p1 = p1->next;
        q1 = q1->next;
    }
    tmp1->next = new point(p1->x, p1->y, nullptr);
    pathFunc(tmp);
    return tmp;
}

void ES(double toiu){
    for(int i = 0; i < numPopulations; ++i) {
        pathLen = pathLength(population[i]);
        path *tmp = mutation(population[i], toiu);
        if(compareTwoPath(tmp, population[i]) > 0){
        // if(tightlyDominantPath(tmp, population[i])){
            population[i] = combination1(tmp, pPath[i]);
            // population[i] = tmp;
        }
        // population[i] = combination1(population[i], gPath);
    }
}

void lastSocial(){
    for(int i = 0; i < numPopulations; ++i){
        for(int j = 0; j < numPopulations; ++j){
            if(population[i] == nullptr || population[j] == nullptr) continue;
            int tmp = tightlyDominantPath(population[i], population[j]);
            if(tmp == 1) population[i] = nullptr;
            else if(tmp == -1) population[j] = nullptr;
        }
    }
    int numPopu = 0;
    for(int i = 0; i < numPopulations; i++){
        if(population[i] == nullptr) continue;
        pPath[numPopu] = makeCopyPath(population[i]);
        population[numPopu] = makeCopyPath(population[i]);
        if(compareTwoPath(pPath[numPopu], gPath) > 0) gPath = makeCopyPath(pPath[numPopu]);
        numPopu++;
    }
    numPopulations = numPopu;
}

void saveExe(){
    decreaseDimension();
    gPath = makeCopyPath(population[0]);
    for(int i = 0; i < numPopulations; ++i){
        pathFunc(population[i]);
        for(int j = 0; j < i; ++j){
            if(population[i] == nullptr || population[j] == nullptr) continue;
            int tmp = compareBadPath(population[i], population[j]);
            // int tmp = tightlyDominantPath(population[j], population[i]);
            if(tmp == 1) 
                population[j] = nullptr;
            else if(tmp == -1)
                population[i] = nullptr;
        }
    }
    int numPopu = 0;
    for(int i = 0; i < numPopulations; i++){
        if(population[i] == nullptr) continue;
        // cout << i << " " << endl;
        pPath[numPopu] = makeCopyPath(population[i]);
        population[numPopu] = makeCopyPath(population[i]);
        if(compareTwoPath(pPath[numPopu], gPath) > 0) gPath = makeCopyPath(pPath[numPopu]);
        numPopu++;
    }
    // if(numPopu > 5)
    numPopulations = numPopu;
    cout << "Number of " << numPopulations << endl;
}

void PSOES(double toiu, int loop){
    int showloop = 20;
    for(int i = 0; i < loop; ++i) {
        est += 0.7/loop;
        toiu += 30.0 / loop;
        // if(i < loop/10) ES(toiu/10);
        if(i < loop/4) PSOmigrate();
        else if(i == loop/4){
            saveExe();
        }
        else {
            ES(toiu);
            PSO();
            updateBestPath();
        }
    }
}