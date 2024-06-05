
struct path {
    double distance=10, angle=10, safety;
    point *begin;
    path() : distance(1000000), angle(1000000), begin(nullptr) {}
    path(point *p1) : distance(1000000), angle(1000000), begin(p1) {}
    path(double distance1, double angle1, point *p1) : distance(distance1), angle(angle1), begin(p1) {}
};

point relative;

double distanceToFinish(double x, double y) {
    point *p = new point(x, y, nullptr);
    return euclideanDistance(p, finish);
}

bool checkValidPosition(point *p){
    int x = (int)p->x, y = (int)p->y;
    // if(x < 0 || y < 0 || x > mapHeight || y > mapWidth) return false;
    if(graphStatus[x][y] > 1000000 || graphStatus[x+1][y] > 1000000 || graphStatus[x][y+1] > 1000000 || graphStatus[x+1][y+1] > 1000000) return false;
    return true;
}

bool checkValidPosition(double x1, double y1){
    int x = (int)x1, y = (int)y1;
    // if(x < 0 || y < 0 || x > mapHeight || y > mapWidth) return false;
    if(graphStatus[x][y] > 1000000 || graphStatus[x+1][y] > 1000000 || graphStatus[x][y+1] > 1000000 || graphStatus[x+1][y+1] > 1000000) return false;
    return true;
}

bool checkValidPoint(point *p){
    int x = (int)p->x, y = (int)p->y;
    if(x < 0 || y < 0 || x > mapHeight || y > mapWidth) return false;
    if(graphStatus[x][y] > 1000000) return false;
    return true;
}

bool checkValidPoint(int x, int y){
    if(x < 0 || y < 0 || x > mapHeight || y > mapWidth) return false;
    if(graphStatus[x][y] > 1000000) return false;
    return true;
}

bool checkValidLine(point *p, point *q){
    double xBegin = p->x, yBegin = p->y, dis = euclideanDistance(p, q);
    double cos = (q->x-xBegin)/dis, sin = (q->y-yBegin)/dis;
    for(double j = 0; j <= dis; ++j)
        if(checkValidPoint((int) (xBegin+j*cos), (int) (yBegin+j*sin)) == false) return false;
    return true;
}

double angleVector(point *p1, point *p2){
    return atan(p2->y - p1->y / p2->x - p1->x);
}

double angleThreePoint(point *p1, point *p2, point *p3){
    return abs(angleVector(p3, p2)- angleVector(p2, p1));
}


void normalLine(point *p1, point *p2, point *p3){
    double y = p3->x - p1->x, x = -(p3->y - p1->y), z = euclideanDistance(p1, p3);
    p2->x += x*2.5/z;
    p2->y += y*2.5/z;
}

pair<double, double> returnNormalLine(point *p1, point *p2, point *p3){
    double y = p3->x - p1->x, x = -(p3->y - p1->y), z = euclideanDistance(p1, p3);
    return make_pair(x, y);
}

double compareTwoPath(path *a, path *b) {
    // return b->angle - a->angle + b->distance - a->distance;
    return (b->angle - a->angle) / b->angle + 2.5 * (b->distance - a->distance) / b->distance;
    // return (b->angle - a->angle) / b->angle + 2.5 * (b->distance - a->distance) / b->distance + (b->safety - a->safety) * 10.5 / b->safety;
}

double compareTwoPath(path *a, path *b, double weight) {
    // return b->angle - a->angle + b->distance - a->distance;
    return weight * (b->angle - a->angle) / b->angle + 2.5 * (b->distance - a->distance) / b->distance;
    // return (b->angle - a->angle) / b->angle + 2.5 * (b->distance - a->distance) / b->distance + (b->safety - a->safety) * 10.5 / b->safety;
}

// int compareTwoPath(path *a, path *b) {
    // return a->angle < b->angle + a->distance < b->distance;
    // return a->distance < b->distance;
    // return a->angle < b->angle;
// }

int compareSamePath(path *a, path *b) {
    double res = abs(a->angle - b->angle)/(a->angle+b->angle) + 1.5*abs(a->distance - b->distance)/(a->distance+b->distance);
    // double res = 2.0 * abs(a->distance - b->distance)/(a->distance+b->distance);
    // cout << "Compare two paths: " << res << endl;
    // if(res < 0.08 || res > 1.1){
    if(res > 0.5) {
        if(compareTwoPath(a, b) > 0) return 1;
        return -1;
    }
    return 0;
}

int compareBadPath(path *a, path *b) {
    double res = abs(a->angle - b->angle)/(a->angle+b->angle) + 1.5*abs(a->distance - b->distance)/(a->distance+b->distance);
    // cout << "Compare two paths: " << res << endl;
    if(res < 0.08) {
        if(compareTwoPath(a, b) > 0) return 1;
        return -1;
    }
    return 0;
}

int dominantPath(path *a, path *b) {
    if(a->angle*1.02 < b->angle && a->distance*1.05 < b->distance && a->safety*1.05 < b->safety) return -1;
    else if(a->angle > b->angle*1.05 && a->distance > b->distance*1.05 && a->safety > b->safety*1.05) return 1;
    return 0;
}

int tightlyDominantPath(path *a, path *b) {
    if(a->angle < b->angle && a->distance < b->distance) return -1;
    else if(a->angle > b->angle && a->distance > b->distance) return 1;
    return 0;
}

pair<double, double> gradientPoint(double x, double y) {
    int x1 = (int) x, y1 = (int) y, cur = graphStatus[x1][y1], ne = -1;
    if(cur < 800000) return make_pair(x, y);
    for(int i = 0; i < 8; ++i) {
        if(graphStatus[x1+nearPoint[i][0]*2][y1+nearPoint[i][1]*2] < cur){
            cur = graphStatus[x1+nearPoint[i][0]*2][y1+nearPoint[i][1]*2];
            ne = i;
        }
    }
    if(ne != -1){
        x += nearPoint[ne][0]*2;
        y += nearPoint[ne][1]*2;
    }
    // pair<double, double> p;
    return make_pair(x, y);
}

pair<double, double> gradientPointTightly(double x, double y) {
    int x1 = (int) x, y1 = (int) y, cur = graphStatus[x1][y1], ne = -1;
    // if(cur < 200000) return make_pair(0, 0);
    for(int i = 0; i < 8; ++i) {
        if(graphStatus[x1+nearPoint[i][0]*2][y1+nearPoint[i][1]*2] < cur){
            cur = graphStatus[x1+nearPoint[i][0]*2][y1+nearPoint[i][1]*2];
            ne = i;
        }
    }
    if(ne == -1) return make_pair(0, 0);
    return make_pair(nearPoint[ne][0],  nearPoint[ne][1]);
}


int pathLength(path *p) {
    point *p1 = p->begin;
    int len = 0;
    while(p1 != nullptr) {
        p1 = p1->next;
        ++len;
    }
    return len;
}

void pathFunc(path *path){
    // cout << "cc";
    point *p = path->begin, *p1 = p->next;
    double x = p1->x - p->x, y = p1->y - p->y;
    double z = sqrt(x*x + y*y);
    double distance = z, angle = 0, pastAngle = acos(x/z) * (y >= 0 ? 1 : -1), currAngle;
    int i = 0;
    while(p->next != nullptr){
        p1 = p->next;
        x = p1->x - p->x;
        y = p1->y - p->y;
        z = sqrt(x*x + y*y);
        distance += z;
        currAngle = 100 * acos(x/z) * (y >= 0 ? 1 : -1);
        if(i++ != 0) angle += pow(currAngle - pastAngle, 2);
        pastAngle = currAngle;
        p = p1;
        // cout << angle << " ";
    }
    path->distance = distance;
    path->angle = max(1.0, angle);
}

void pathFunc1(path *path){
    // cout << endl;
    point *p = path->begin, *p1 = p->next;
    double x = p1->x - p->x, y = p1->y - p->y;
    double z = sqrt(x*x + y*y);
    double distance = z, angle = 0, pastAngle = acos(x/z) * (y >= 0 ? 1 : -1), currAngle;
    path->safety = 0;
    while(p->next != nullptr){
        path->safety += graphStatus[(int)p->x][(int)p->y];
        p1 = p->next;
        x = p1->x - p->x;
        y = p1->y - p->y;
        z = sqrt(x*x + y*y);
        distance += z;
        distance += max(0, graphStatus[(int)p1->x][(int)p1->y]-500000);
        currAngle = acos(x/z) * (y >= 0 ? 1 : -1);
        // cout << currAngle << " " << (x/z) << endl;
        angle += pow(currAngle - pastAngle, 1.9);
        pastAngle = currAngle;
        p = p1;
        // cout << angle << " ";
    }
    path->safety /= pathLength(path);
    path->distance = distance;
    path->angle = angle;
}

