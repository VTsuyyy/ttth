
struct point{
    double x, y;
    point *next;    
    point() : x(0), y(0), next(nullptr) {};
    point(double x1, double y1, point *next1) : x(x1), y(y1), next(next1) {}
};
int nearPoint[8][2] = {{1, 0}, {0, 1}, {1, 1}, {-1, 1}, {1, -1}, {0, -1}, {-1, 0}, {-1, -1}};
point *start, *finish, *obstacles[1000], *initStart, *initFinish;
int mapHeight, mapWidth, numObstacle;
int graphStatus[1000][1000] = {};

void inputGraph(string fp, int numInput){
    fp = "input\\map" + to_string(numInput) + ".txt";
	freopen(&fp[0], "r", stdin);

    start = new point();
    finish = new point();
    cin >> mapHeight >> mapWidth;
    cin >> start->x >> start->y >> finish->x >> finish->y;
    cin >> numObstacle;
    start->x *= 2;
    start->y *= 2;
    finish->x *= 2;
    finish->y *= 2;
    mapHeight *= 2;
    mapWidth *= 2;

    string s;
    for(int i = 0; i < numObstacle; ++i){
        cin.ignore();
        getline(cin, s);
        // cout << s << endl;
        stringstream ss(s);
        double x1, y1;
        while(ss >> x1){
            ss >> y1;
            x1 *= 2;
            y1 *= 2;
            obstacles[i] = new point(x1, y1, obstacles[i]);
        }
    }
    obstacles[numObstacle] = new point(-0, -0, obstacles[numObstacle]);
    obstacles[numObstacle] = new point(mapWidth-0, -0, obstacles[numObstacle]);
    obstacles[numObstacle] = new point(mapWidth-0, mapHeight-0, obstacles[numObstacle]);
    obstacles[numObstacle] = new point(-0, mapHeight-0, obstacles[numObstacle]);
    ++numObstacle;
}

double euclideanDistance(point *p1, point *p2){
    double x = p1->x - p2->x, y = p1->y - p2->y;
    return sqrt(x*x + y*y);
}

bool vectorThreePoint(point *p1, point *p2, point *p3){
    return (p3->x-p2->x)*(p2->x-p1->x) + (p3->y-p2->y)*(p2->y-p1->y) > 0.0;
}

void changeToNewGraph(){
    initStart = new point(start->x, start->y, nullptr);
    initFinish = new point(finish->x, finish->y, nullptr);
    double a = finish->x-start->x, b = finish->y-start->y;
    double cos = a/sqrt(a*a+b*b), sin = b/sqrt(a*a+b*b);
    for(int i = 0; i < numObstacle; i++){
        point *p = obstacles[i];
        while(p) {
            double x1 = cos * (p->x - start->x) + sin * (p->y - start->y);
            double y1 = cos * (p->y - start->y) - sin * (p->x - start->x);
            p->x = x1+150;
            p->y = y1+150;
            p = p->next;
        }
    }
    finish->x = euclideanDistance(start, finish)+150;
    finish->y = 150;
    start->x = 150;
    start->y = 150;
}

void changeToInitGraph(){
    start = initStart;
    finish = initFinish;
    double a = finish->x-start->x, b = finish->y-start->y;
    double cos = a/sqrt(a*a+b*b), sin = b/sqrt(a*a+b*b);
    for(int i = 0; i < numObstacle; i++){
        point *p = obstacles[i];
        while(p) {
            p->x -= 150;
            p->y -= 150;
            double x1 = cos * p->x - sin * p->y;
            double y1 = cos * p->y + sin * p->x;
            p->x = x1 + start->x;
            p->y = y1 + start->y;
            p = p->next;
        }
    }
}

bool onMapSize(int i, int j){
    if(i < 0 || j < 0 || i > mapWidth || j > mapHeight) return false;
    return true;
}

void markPointNotCome(double x, double y){
    int x1 = max(1, (int) x), y1 = max(1, (int) y);
    graphStatus[x1][y1] = 1000000;
    graphStatus[x1-1][y1] = 1000000;
    graphStatus[x1][y1-1] = 1000000;
    graphStatus[x1-1][y1-1] = 1000000;
    graphStatus[x1+1][y1-1] = 1000000;
    graphStatus[x1-1][y1+1] = 1000000;
    graphStatus[x1+1][y1] = 1000000;
    graphStatus[x1][y1+1] = 1000000;
    graphStatus[x1+1][y1+1] = 1000000;
}

void markPointNotCome2(double x, double y){
    int x1 = max(1, (int) x), y1 = max(1, (int) y);
    graphStatus[x1-2][y1] = 1000000;
    graphStatus[x1][y1-2] = 1000000;
    graphStatus[x1-2][y1-2] = 1000000;
    graphStatus[x1+2][y1-2] = 1000000;
    graphStatus[x1-2][y1+2] = 1000000;
    graphStatus[x1+2][y1] = 1000000;
    graphStatus[x1][y1+2] = 1000000;
    graphStatus[x1+2][y1+2] = 1000000;
}


int reSizeX;
int convx(double a) {
	return (int)(a * reSizeX);
}

int reSizeY;
int convy(double a) {
	return 900-(int)(a * reSizeY);
}
void markLineNotCome(point *p, point *q){
    // cout << p->x << " " << p->y << " " << q->x << " " << q->y << endl;
    double xBegin = p->x, yBegin = p->y, dis = euclideanDistance(p, q);
    double cos = (q->x-xBegin)/dis, sin = (q->y-yBegin)/dis;
    for(double j = 0; j < dis+1.0; ++j){
        markPointNotCome(xBegin+j*cos, yBegin+j*sin);
    }
}

// int stopCome = 50000;
// void comePoint(int x, int y){
//     if(--stopCome < 0 || graphStatus[x][y] != 0) return;
//     graphStatus[x][y] = 1;
//     for(int i = 0; i < 8; ++i)
//         comePoint(x+nearPoint[i][0], y+nearPoint[i][1]);
// }

void smoothObstacle(){
    // comePoint((int)start->x, (int)start->y);
    queue<pair<int,int> > pq;
    pq.push(make_pair((int)start->x, (int)start->y));
    while(pq.empty() == false) {
        int x = pq.front().first, y = pq.front().second; 
        pq.pop();
        for(int i = 0; i < 8; ++i){
            if(graphStatus[x+nearPoint[i][0]][ y+nearPoint[i][1]] == 0){
                pq.push(make_pair(x+nearPoint[i][0], y+nearPoint[i][1]));        
                graphStatus[x+nearPoint[i][0]][ y+nearPoint[i][1]] = 1;    
            }
        }
    }
}

void obstacleGradient(){
    queue<pair<int,int>> q;
    for(int i = 0; i <= mapHeight; i++)
        for(int j = 0; j <= mapWidth; j++) 
            if(graphStatus[i][j] == 1000000){
                q.push(make_pair(i,j));
                graphStatus[i][j] = 1100100;
            }
    while(q.empty() == false){
        int i = q.front().first, j = q.front().second;
        q.pop();
        for(int k = 0; k < 8; ++k){
            int i1 = i+nearPoint[k][0], j1 = j+nearPoint[k][1], t = 1;
            if(onMapSize(i1, j1) == false) continue;
            if(graphStatus[i1][j1] == 0) graphStatus[i1][j1] = graphStatus[i][j] + 100000;
            else if(graphStatus[i1][j1] == 1) graphStatus[i1][j1] = max(graphStatus[i][j]-100000, 10);
            else t = 0;
            if(t) q.push(make_pair(i1, j1));
        }
    }

}

void markObstacle(){
    reSizeX = 1000 / mapWidth;
    reSizeY = 1000 / mapHeight;
    for(int i = 0; i < numObstacle; ++i){
        point *p = obstacles[i], *q = p->next;
        while(q != nullptr){
            // if(q->next != nullptr && vectorThreePoint(p, q, q->next) == false){
            //     markPointNotCome2(q->x, q->y);
            // }
            markLineNotCome(p, q);
            p = q;
            q = q->next;
        }
        q = obstacles[i];
        markLineNotCome(p, q);
    }
    // for(int i = 0; i < 300; i++){
    //     for(int j = 0; j < 300; j++)
    //         cout << !graphStatus[i][j] << " ";
    //     cout << endl;
    // }
    graphStatus[(int)start->x][(int)start->y] = 0;
    graphStatus[(int)finish->x][(int)finish->y] = 0;
    smoothObstacle();

    obstacleGradient();
}

void resetGraphStatus(){
    for(int i = 0; i < 1000; ++i) {
        // rand();
        for(int j = 0; j < 1000; ++j)
            graphStatus[i][j] = 0;
    }
    markObstacle();
}
