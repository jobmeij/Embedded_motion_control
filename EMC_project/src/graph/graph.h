#include "../vector2.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "../config.h"
#include "../objects.h"

#ifndef graph_H
#define graph_H

using namespace std;

static unsigned int defaultcolor[] = {255,0,0};

class Graph{
private:
    double scale;
    double offset;
    unsigned long width;
    unsigned long height;
    cv::Mat graph;

public:
    enum drawtypes {points, lines};
    Graph(double scale_, double offset_, unsigned long width_, unsigned long height_);
    void addData(std::vector<float>& data, drawtypes drawtype_ = lines, unsigned int color[3] = defaultcolor);
    void emptygraph();
    void show();
};

#endif //graph_H
