#include<bits/stdc++.h>
using namespace std;
struct point{
    double x, y;
    point *next;    
    point() : x(0), y(0), next(nullptr) {};
    point(double x1, double y1, point *next1) : x(x1), y(y1), next(next1) {}
};
struct path {
    double distance, angle, safety;
    point *begin;
    path() : distance(1000000), angle(1000000), begin(nullptr) {}
    path(point *p1) : distance(1000000), angle(1000000), begin(p1) {}
    path(double distance1, double angle1, point *p1) : distance(distance1), angle(angle1), begin(p1) {}
};
path *paths[50];
point *start, *finish, *obstacles[1000], *population[100];
int mapHeight, mapWidth, numObstacle, numPath = 0;

bool check(point *m, point *n, point *p, point *q){
    double b = n->x - m->x, a = m->y - n->y, c = a * m->x + b * m->y;
    return  (p->x * a + p->y * b - c) * (q->x * a + q->y * b - c) <= 0;
}

double distancePointToLine(point *p, point *m, point *n){
    double a = sqrt(pow(m->x - p->x, 2) + pow(m->y - p->y, 2)), b = sqrt(pow(n->x - p->x, 2) + pow(n->y - p->y, 2)), c = sqrt(pow(m->x - n->x, 2) + pow(m->y - n->y, 2));
    double q = (a+b+c)/2, s = sqrt(q*(q-a)*(q-b)*(q-c)), h = s*2.0 / c;
    // cout << a << " " << b << " " << c << endl;
    if(abs(sqrt(a*a-h*h)+sqrt(b*b-h*h)-c) > 0.01) return min(a, b);
    // cout << "1";
    return h;
}

double distanceToObstacle(point *p, point *p1){
    double tmp = 1e9;
    for(int i = 0; i < numObstacle; ++i){
        point *obstacle = obstacles[i];
        while(obstacle->next != nullptr){
            tmp = min(tmp, distancePointToLine(p, obstacle, obstacle->next));
            tmp = min(tmp, distancePointToLine(p1, obstacle, obstacle->next));
            tmp = min(tmp, distancePointToLine(obstacle, p, p1));
            tmp = min(tmp, distancePointToLine(obstacle->next, p, p1));
            obstacle = obstacle->next;
        }
        double tmp = 1e9;
        tmp = min(tmp, distancePointToLine(p, obstacle, obstacles[i]));
        tmp = min(tmp, distancePointToLine(p1, obstacle, obstacles[i]));
        tmp = min(tmp, distancePointToLine(obstacle, p, p1));
        tmp = min(tmp, distancePointToLine(obstacles[i], p, p1));
    }
    // cout << tmp << " ";
    return tmp;
}

void pathFunc(path *path1) {
    point *p = path1->begin, *p1 = p->next;
    double x = p1->x - p->x, y = p1->y - p->y;
    double z = sqrt(x*x + y*y);
    double distance = z, angle = 0, pastAngle = acos(x/z) * (y >= 0 ? 1 : -1), currAngle, safety = 0;
    int i = 0;
    while(p->next != nullptr){
        p1 = p->next;
        x = p1->x - p->x;
        y = p1->y - p->y;
        z = sqrt(x*x + y*y);
        distance += z;
        double tmp = 100.0/pow(distanceToObstacle(p, p1), 2);
        // cout << tmp << " ";
        safety += tmp;
        currAngle = 100 * acos(x/z) * (y >= 0 ? 1 : -1);
        if(i++ != 0) angle += abs(currAngle - pastAngle);
        pastAngle = currAngle;
        p = p1;
    }
    // cout << endl;
    path1->safety = safety/i;
    path1->distance = distance;
    path1->angle = max(1.0, angle) / sqrt(i);
}

bool checkValidPath(path *p){
    point *p1 = p->begin;
    if(abs(p1->x - start->x) > 0.1 || abs(p1->y - start->y) > 0.1) {
        p1 = new point(start->x, start->y, p1);
    }
    while(true){
        if(p1->next == nullptr){
            if(abs(p1->x - finish->x) < 0.1 && abs(p1->y - finish->y) < 0.1) break;
            p1->next = new point(finish->x, finish->y, nullptr);
        }
        for(int i = 0; i < numObstacle; ++i){
            point *obstacle = obstacles[i];
            while(obstacle->next != nullptr){
                if(check(p1, p1->next, obstacle, obstacle->next) && check(obstacle, obstacle->next, p1, p1->next)) {
                    return false;    
                }
                obstacle = obstacle->next;
            }
            if(check(p1, p1->next, obstacle, obstacles[i]) && check(obstacle, obstacles[i], p1, p1->next)) {
                return false;
            }
        }
        p1 = p1->next;
    }
    pathFunc(p);
    if(p->safety > 300) return false;
    return true;
} 

int main(){
    // freopen("input\\map6.txt", "r", stdin);
    // const string s = 
    int numIO = 4;
    ifstream file1("HaiNSGAII\\nsgaii_test"+to_string(numIO)+".txt");
    // ifstream file1("HaiPSOES\\psoes_test"+to_string(numIO)+".txt");
    // ifstream file1("HaiPSO\\pso_test"+to_string(numIO)+".txt");
    // ifstream file1("output\\out"+to_string(numIO)+".txt");
    ifstream file2("input\\map"+to_string(numIO)+".txt");
    // freopen("out.txt", "r", stdin);
    start = new point();
    finish = new point();
    file2 >> mapHeight >> mapWidth;
    file2 >> start->x >> start->y >> finish->x >> finish->y;
    file2 >> numObstacle;

    string s;
    for(int i = 0; i < numObstacle; ++i){
        file2.ignore();
        getline(file2, s);
        stringstream ss(s);
        double x1, y1;
        while(ss >> x1){
            ss >> y1;
            obstacles[i] = new point(x1, y1, obstacles[i]);
        }
    }
    for(int i = 0; i < numObstacle; ++i){
        point *p = obstacles[i], *q = p->next;
        while(q != NULL){
            // cout << p->x << " " << p->y << " " <<  q->x << " " << q->y << endl;
            p = p->next;
            q = q->next;
        }
        // cout << "-----" << endl;
    }
    
    file2.close();
    double x=1, y;
    file1 >> x;
    int loop = 1000;
    while(loop-- > 0){
        // cout << x;
        if(abs(x+2.0) < 0.01 || abs(x+1.0) < 0.01){
            path *tmp = new path();
            point *p = population[numPath], *q = p->next, *l = q->next;
            p->next = nullptr;
            while(l != nullptr){
                q->next = p;
                p = q;
                q = l;
                l = l->next;
            }
            q->next = p;
            population[numPath] = q;
            tmp->begin = population[numPath];
            if(checkValidPath(tmp)) {
                // pathFunc(tmp);
                paths[numPath] = tmp;
                ++numPath;
            }
            else population[numPath] = nullptr;
            if(abs(x+2.0) < 0.01) break;
            file1 >> x;
            continue;
        }
        file1 >> y;
        // cout << x << " " << y << endl;
        population[numPath] = new point(x, y, population[numPath]);
        file1 >> x;
    }
    file1.close();
    // cout << numPath << endl;
    sort(paths, paths+numPath, [](path *path1, path *path2)->bool{
        return path1->distance < path2->distance;
    });
    map<int, int> ma;
    ma[0] = 0;
    ma[100] = 0;
    cout << "map " << numIO << endl;
    for(int i = 0; i < numPath; ++i){
        cout << "path " << i << ": " << paths[i]->distance << " " << paths[i]->angle << " " << paths[i]->safety << endl;
    }

    // point *tt = new point(44.831607137124756, 65.23612244242828, nullptr), *tt1 = new point(54.70618961196313, 60.37850241367214, nullptr);
    // point *o1 = new point(51, 62, nullptr), *o2 = new point(57, 55, nullptr);
    // // cout << endl << distanceToObstacle(tt, tt1);
    // cout << distancePointToLine(tt, o1, o2) << endl;
    // cout << distancePointToLine(tt1, o1, o2) << endl;
}