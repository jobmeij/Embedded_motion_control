#include "./vector2.h"
#include <vector>
#include <string.h>
#include <iostream>

#ifndef objects_H
#define objects_H

using namespace std;

struct Position{
    float x;
    float y;
    float a;
    Position(){this->x = 0; this->y = 0; this->a = 0;}
    Position(float x_, float y_, float a_){this->x = x_; this->y = y_; this->a = a_;}
    Position operator- ()const{
        Position p;
        p.x = -this->x;
        p.y = -this->y;
        p.a = -this->a;
        return p;
    }
    Position operator- (const Position& other)const{
        Position p;
        p.x = this->x-other.x;
        p.y = this->y-other.y;
        p.a = this->a-other.a;
        return p;
    }
};

struct link;

struct node{
    Vector2 location;
    float angle;
    vector<link*> nodelinks;
    enum properties {start, destination, normal} property;
    node(Vector2 location_, float angle_, properties property_){location = location_; angle = angle_; property = property_;}
    node(){location = Vector2(0,0); angle = 0; property = properties::normal;}

    void transform(float x_, float y_, float a_){
        this->location.transform(x_, y_, a_);
        this->angle + a_;
    }

    void scale(float x_, float y_){
        this->location.scale(x_, y_);
    }
};

struct link{
    node* node1;
    node* node2;
    bool obstructed;
    float distance;
    link(node* node1_, node* node2_, bool obstructed_){
        node1 = node1_;
        node2 = node2_;
        node1->nodelinks.push_back(this);
        node2->nodelinks.push_back(this);
        obstructed = obstructed_;
        distance = node1_->location.distance(node2_->location);
    }
    link(){node1 = NULL; node2 = NULL; obstructed = false; distance = 0;}
};

namespace Objectoptions {
    enum connections {open = -1, closed = 0};
    enum drawtypes {points, lines};
}

struct Object;

struct Point{
    Vector2 location;
    float weight;
    vector<Point*> connectedwith;
    Point* projectionpoint;
    vector<Object*> parents;
    enum properties {floating, connected, concave, convex, projected} property;

    Point(){
        location = Vector2(0,0);
        weight = 0;property = floating;
        connectedwith.clear();
        projectionpoint = NULL;
        parents.clear();
    }

    Point(Vector2 location_,float weight_, properties property_){
        location.x = location_.x;
        location.y = location_.y;
        weight = weight_;
        property = property_;
        connectedwith.clear();
        projectionpoint = NULL;
        parents.clear();
    }

    bool sameparentobject(const Point &other)const{
        bool sameparents = false;
        for(unsigned long ti = 0; ti < this->parents.size(); ti++){
            for(unsigned long oi = 0; oi < other.parents.size(); oi++){
                if(this->parents[ti] == other.parents[oi]){
                    sameparents = true;
                }
            }
        }
        return sameparents;
    }

    bool connectedpoint(const Point &other)const{
        bool connectedpoints = false;
        for(int cp = 0; cp < this->connectedwith.size(); cp++){
            if(this->connectedwith[cp] == &other){
                connectedpoints = true;
            }
        }
        return connectedpoints;
    }

};

typedef std::vector<Point> Points;
typedef std::vector<Point*> Pointsref;
typedef unsigned int Color[3];
typedef char Name[15];

struct Object{

    enum types {wall, door, staticobstacle, dynamicobstacle, robot, origin, test, safeDis, destination, projections, node, nodes, cabinet, roomwall, path, start} type;

    Name name;
    float pointradius;
    Points points;
    Pointsref projectedpoints;
    Color color;
    Objectoptions::connections connection;
    Objectoptions::drawtypes drawtype;
    bool remove;
    bool newobject;

    Object(types type_, const Name &name_, const Color &color_, Objectoptions::drawtypes drawtype_, Objectoptions::connections connection_){
        type = type_;
        strcpy(name, name_);
        copy(color_,color_+3,color);
        drawtype = drawtype_;
        connection = connection_;
        remove = false;
        newobject = true;
        pointradius = 1;
        points.clear();
        projectedpoints.clear();
    }

    float anglefrom(const Point& p)const{
        float angle = -1;
        if(points.size() == 2){ //only relevant if object has 2 points [wall / door]
            Vector2 p1 = p.location;
            Vector2 p2;
            (p1 == points[0].location) ? (p2 = points[1].location) : (p2 = points[0].location);
            Vector2 diff = p2 - p1;
            angle = atan2(diff.y,diff.x)+M_PI;
        }
        return angle;
    }

    float angle()const{
        float angle = -1;
        if(points.size() == 2){ //only relevant if object has 2 points [wall / door]
            Vector2 p1 = points[0].location;
            Vector2 p2 = points[1].location;
            Vector2 diff = p1 - p2;
            angle = fmod(atan2(diff.y,diff.x)+M_PI,M_PI);
        }
        return angle;
    }

    float length()const{
        float length = -1;
        if(points.size() == 2){ //only relevant if object has 2 points [wall / door]
            Vector2 p1 = points[0].location;
            Vector2 p2 = points[1].location;
            Vector2 diff = p1 - p2;
            length = diff.length();
        }
        return length;
    }

    Vector2 middle()const{
        Vector2 mid(0,0);
        for(unsigned long i = 0; i < this->points.size(); i++){
            mid = mid + this->points[i].location;
        }
        mid = mid / this->points.size();
        return mid;
    }

    float smallestrelativeangle(const Object& other)const{
        float a1 = this->angle();
        float a2 = other.angle();
        if(a1 != -1 && a2 != -1){
            float adiff1 = abs(a1 - a2);
            float adiff2 = abs(M_PI - adiff1);
            if(adiff1 <= adiff2){
                return adiff1;
            }else{
                return adiff2;
            }
        }else{
            return -1;
        }
    }

    float anglebetween(const Object& other)const{
        float angle = -1;
        if(this->points.size() == 2 && other.points.size() == 2){
            const Point& O1p1 = this->points[0];
            const Point& O1p2 = this->points[1];
            const Point& O2p1 = other.points[0];
            const Point& O2p2 = other.points[1];
            Vector2 v1;
            Vector2 v2;
            Vector2 v3;
            bool link = false;
            if(O1p1.location == O2p1.location){v1 = O1p2.location; v2 = O1p1.location; v3 = O2p2.location; link = true;}
            if(O1p1.location == O2p2.location){v1 = O1p2.location; v2 = O1p1.location; v3 = O2p1.location; link = true;}
            if(O1p2.location == O2p1.location){v1 = O1p1.location; v2 = O1p2.location; v3 = O2p2.location; link = true;}
            if(O1p2.location == O2p2.location){v1 = O1p1.location; v2 = O1p2.location; v3 = O2p1.location; link = true;}
            if(link){
                Vector2 l1 = v1 - v2;
                Vector2 l2 = v3 - v2;
                float a1 = (l1.angle() - l2.angle());
                (a1 > 0) ? (angle = a1) : (angle = a1+2*M_PI);
            }
        }
        return angle;
    }

    float averageperpendiculardistance(const Object& other)const{
        float dist = -1;
        float d1 = 0;
        float d2 = 0;
        if(this->points.size() == 2 && other.points.size() == 2){ //only relevant if object has 2 points [wall / door]
            Vector2 offset = this->points[0].location;
            Vector2 v1 = other.points[0].location - offset;
            Vector2 v2 = this->points[1].location - offset;
            Vector2 v3 = v2.unit()*v1.dot(v2.unit());
            Vector2 v4 = v1-v3;
            d1 = v4.length();
            v1 = other.points[1].location - offset;
            v3 = v2.unit()*v1.dot(v2.unit());
            v4 = v1-v3;
            d2 = v4.length();
            dist = (d1 + d2) / 2;
        }
        return dist;
    }

    Vector2 projection(const Point& other){
        Vector2 v(0,0);
        if(this->points.size() == 2){
            Vector2 offset = this->points[0].location;
            Vector2 v1 = other.location - offset;
            Vector2 v2 = this->points[1].location - offset;
            float projection = v1.dot(v2.unit());
            Vector2 v3 = v2.unit()*projection;
            //if(projection >= 0 && projection <= v2.length()){
            v = v3 + offset;
            //}
        }
        return v;
    }

    Vector2 constrainedprojection(const Point& other, float minoffset){
        Vector2 v(0,0);
        if(this->points.size() == 2){
            Vector2 offset = this->points[0].location;
            Vector2 v1 = other.location - offset;
            Vector2 v2 = this->points[1].location - offset;
            float projection = v1.dot(v2.unit());
            Vector2 v3 = v2.unit()*projection;
            if(projection >= minoffset && projection <= v2.length() - minoffset){
                v = v3 + offset;
            }
        }
        return v;
    }

    float gapdistance(const Object& other)const{
        float dist = -1;
        if(points.size() == 2  && other.points.size() == 2){ //only relevant if object has 2 points [wall / door]
            float obj1length = this->length();
            Vector2 obj1p1 = this->points[0].location;
            Vector2 obj1p2 = this->points[1].location;
            Vector2 obj2p1 = other.points[0].location;
            Vector2 obj2p2 = other.points[1].location;
            float d11_21 = obj1p1.distance(obj2p1);
            float d12_21 = obj1p2.distance(obj2p1);
            float d11_22 = obj1p1.distance(obj2p2);
            float d12_22 = obj1p2.distance(obj2p2);
            if(d11_21 <= obj1length && d12_21 <= obj1length || d11_22 <= obj1length && d12_22 <= obj1length){
                dist = 0;
            }else{
                dist = d11_21;
                if(d12_21 < dist){dist = d12_21;}
                if(d11_22 < dist){dist = d11_22;}
                if(d12_22 < dist){dist = d12_22;}
            }
        }
        return dist;
    }

    bool connectedto(const Object& other)const{
        bool connectedobjects = false;
        for(unsigned long p1 = 0; p1 < this->points.size(); p1++){
            for(unsigned long p2 = 0; p2 < other.points.size(); p2++){
                const Point& point1 = this->points[p1];
                const Point& point2 = other.points[p2];
                for(unsigned long pa1 = 0; pa1 < point1.parents.size(); pa1++){
                    for(unsigned long pa2 = 0; pa2 < point2.parents.size(); pa2++){
                        if(point1.parents[pa1] == point2.parents[pa2]){
                            connectedobjects = true;
                        }
                    }
                }

            }
        }
        return connectedobjects;
    }

    void transform(float x_, float y_, float a_){
        for(unsigned long p = 0; p < this->points.size(); p++){
            Vector2& point = this->points[p].location;
            point.transform(x_, y_, a_);
        }
    }

    void scale(float x_, float y_){
        for(unsigned long p = 0; p < this->points.size(); p++){
            Vector2& point = this->points[p].location;
            point.scale(x_, y_);
        }
    }


};

struct Map{

    std::vector<Object> objects;

    void transform(float x_, float y_, float a_){
        for(unsigned long i = 0; i < objects.size(); i++){
            Object& obj = objects[i];
            if(obj.type != Object::robot){
                obj.transform(x_, y_, a_);
            }
        }
    }

    void scale(float x_, float y_){
        for(unsigned long i = 0; i < objects.size(); i++){
            Object& obj = objects[i];
            if(obj.type != Object::robot){
                obj.scale(x_, y_);
            }
        }
    }

    void print(){
        for(unsigned long i = 0; i < this->objects.size(); i++){
            cout << this->objects[i].name << ", ";
        }
        cout << endl;
    }

    void printparents(){
        cout << "parents" << endl;
        for(unsigned long i = 0; i < objects.size(); i++){
            Object& obj = objects[i];
            cout << obj.name << " :" << &obj << endl;
            for(unsigned long p = 0; p < obj.points.size(); p++){
                Point& point = obj.points[p];
                cout << "Point:" << p << "-" << &point;
                for(unsigned long pa = 0; pa < point.parents.size(); pa++){
                    cout << " >> parent:" << point.parents[pa] << " ";
                }
                cout << endl;
            }
            cout << endl;
        }
    }

    void printprojectionlinks(){
        cout << "Projections" << endl;
        for(unsigned long i = 0; i < objects.size(); i++){
            Object& obj = objects[i];
            cout << obj.name << endl;
            for(unsigned long p = 0; p < obj.points.size(); p++){
                Point& point = obj.points[p];
                cout << p << ": projection:" << point.projectionpoint << " " << endl;
            }
            cout << endl;
        }
    }

    void removeobjects(){
        if(this->objects.size() > 0){
            unsigned long r = this->objects.size()-1;
            for(unsigned long i = this->objects.size()-1; i > 0; i--){
                Object obj = this->objects[i];
                if(obj.remove){
                    Object temp = this->objects[r];
                    this->objects[r] = obj;
                    this->objects[i] = temp;
                    r--;
                }
            }
            unsigned int c = 0;
            for(unsigned long i = this->objects.size()-1; i > 0; i--){
                Object& obj = this->objects[i];
                if(obj.remove){
                    this->objects.pop_back();
                    c++;
                }
            }
        }
    }

    void setobjectsold(){
        for(unsigned long i = 0; i < this->objects.size(); i++){
            Object& obj = this->objects[i];
            obj.newobject = false;
        }
    }
};

#endif //objects_H
