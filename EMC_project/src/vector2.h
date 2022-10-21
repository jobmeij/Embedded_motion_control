#include <cmath>

#ifndef Vector2_H
#define Vector2_H

using namespace std;

struct Vector2{
    float x;
    float y;
    Vector2(float x_, float y_){
        x = x_;
        y = y_;
    }
    Vector2(){
        x = 0;
        y = 0;
    }
    Vector2 operator+= (const Vector2& other)const{
        Vector2 v(0,0);
        v.x = this->x + other.x;
        v.y = this->y + other.y;
        return v;
    }
    Vector2 operator+ (const Vector2& other)const{
        Vector2 v(0,0);
        v.x = this->x + other.x;
        v.y = this->y + other.y;
        return v;
    }
    Vector2 operator- (const Vector2& other)const{
        Vector2 v(0,0);
        v.x = this->x - other.x;
        v.y = this->y - other.y;
        return v;
    }
    Vector2 operator/ (float s)const{
        Vector2 v(0,0);
        v.x = this->x/s;
        v.y = this->y/s;
        return v;
    }
    Vector2 operator* (float s)const{
        Vector2 v(0,0);
        v.x = this->x*s;
        v.y = this->y*s;
        return v;
    }
    bool operator==(const Vector2& other)const{
        return (this->x == other.x && this->y == other.y);
    }
    float distance(const Vector2 &other)const{
        return sqrt(pow((this->x-other.x),2) + pow((this->y-other.y),2));
    }
    float length()const{
        Vector2 v(this->x,this->y);
        return sqrt(v.dot(v));
    }
    float angle()const{
        return atan2(this->y,this->x)+M_PI;
    }
    float dot(const Vector2 &other)const{
        return ((this->x * other.x) + (this->y * other.y));
    }
    Vector2 unit()const{
        Vector2 v(this->x,this->y);
        if(v.length() != 0){
            v = v/v.length();
        }else{
            v.x = 0;
            v.y = 0;
        }
        return v;
    }
    void transform(float x_, float y_, float a_){
        float xtemp = this->x;
        float ytemp = this->y;
        this->x = xtemp*cos(a_) - ytemp*sin(a_) + x_;
        this->y = xtemp*sin(a_) + ytemp*cos(a_) + y_;
    }
    void scale(float x_, float y_){
        float xtemp = this->x;
        float ytemp = this->y;
        this->x = xtemp*x_;
        this->y = ytemp*y_;
    }




};

#endif //Vector2_H
