
bool initRandPath(point *p, point *q){
    double xBegin = p->x, yBegin = p->y, dis = euclideanDistance(p, q);
    double cos = (q->x-xBegin)/dis, sin = (q->y-yBegin)/dis;
    int rrr = rand()%40-25;
    // rrr=0;
    for(double j = 0; j < dis; j += 20){
        // double tmp, tmp1;
        p->next = new point(max(3.0, xBegin+j*cos-rrr*sin*(1-pow((j-dis/2)/dis, 2))), max(3.0, yBegin+j*sin+rrr*cos*(1-pow((j-dis/2)/dis, 2))), nullptr);
        // line(img, Point(convx(p->x), convy(p->y)), Point(convx(p->next->x), convy(p->next->y)), Scalar((120, 0, 255), 1));
        p = p->next;
    }
    p->next = q;
    return (rrr < 0);
    // line(img, Point(convx(p->x), convy(p->y)), Point(convx(p->next->x), convy(p->next->y)), Scalar((120, 0, 255), 1));
}