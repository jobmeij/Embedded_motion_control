#include "../vector2.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "../config.h"
#include "../objects.h"
#include <string>
#include "../world_model/world_model.h"

#ifndef visualisation_H
#define visualisation_H

using namespace std;
using namespace Objectoptions;

class Visualisation{
private:
    double resolution;
    cv::Point2d canvas_center;
    cv::Mat canvas;
    unsigned long width;
    unsigned long height;
    Position origin;

public:
    Visualisation(unsigned long width_, unsigned long height_);
    void addObject(Object& obj);
    void addMap(Map& obj);
    void addrobot();
    void addSafeDis(close_prx safe, Position T);
    void emptycanvas();
    void visualize();
    void setorigin(Position origin_);
    cv::Point2d worldToCanvas(const Vector2& p);
};

#endif //visualisation_H
