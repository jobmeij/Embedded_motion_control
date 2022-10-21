#include "objects.h"

#ifndef Instances_H
#define Instances_H

Object originobj(){
    Color color = {0, 0, 0};
    Name name = "O";
    Object origin(Object::origin, name, color, Objectoptions::lines, Objectoptions::open);
    origin.points.push_back(Point(Vector2( 1 ,  0),0,Point::connected));
    origin.points.push_back(Point(Vector2( 0 ,  0),0,Point::connected));
    origin.points.push_back(Point(Vector2( 0 ,  1),0,Point::connected));
    return origin;
}

Object robotobj(){
    Color color = {255, 0, 0};
    Name name = "R";
    Object robot(Object::robot, name, color, Objectoptions::lines, Objectoptions::closed);
    robot.points.push_back(Point(Vector2( 0.1 , -0.2),0,Point::connected));
    robot.points.push_back(Point(Vector2( 0.1 , -0.1),0,Point::connected));
    robot.points.push_back(Point(Vector2( 0.05, -0.1),0,Point::connected));
    robot.points.push_back(Point(Vector2( 0.05,  0.1),0,Point::connected));
    robot.points.push_back(Point(Vector2( 0.1 ,  0.1),0,Point::connected));
    robot.points.push_back(Point(Vector2( 0.1 ,  0.2),0,Point::connected));
    robot.points.push_back(Point(Vector2(-0.1 ,  0.2),0,Point::connected));
    robot.points.push_back(Point(Vector2(-0.1 , -0.2),0,Point::connected));
    return robot;
}

Object arrowobj(){
    Color color = {255, 255, 255};
    Name name = "Dest";
    Object destination(Object::destination, name, color, Objectoptions::lines, Objectoptions::open);
    destination.points.push_back(Point(Vector2(-0.2, -0.1),0,Point::connected));
    destination.points.push_back(Point(Vector2(0, 0),0,Point::connected));
    destination.points.push_back(Point(Vector2(-0.2, 0.1),0,Point::connected));
    destination.points.push_back(Point(Vector2(0, 0),0,Point::connected));
    destination.points.push_back(Point(Vector2(-0.6, 0),0,Point::connected));
    return destination;
}

#endif //Instances_H
