#include "graph.h"

Graph::Graph(double scale_, double offset_, unsigned long width_, unsigned long height_){
    scale = scale_;
    offset = offset_;
    width = width_;
    height = height_;
    emptygraph();
}

void Graph::addData(std::vector<float>& data, Graph::drawtypes drawtype, unsigned int color[3]){
    cv::Point2d p1;
    cv::Point2d p2;
    switch(drawtype){
    case lines :{
        if (data.size() > 1){
            for(unsigned int i = 0; i < data.size()-1; ++i){
                unsigned int j = (i + 1) % data.size();
                p1 = cv::Point2d(i, (-data[i] * scale)-offset+height);
                p2 = cv::Point2d(j, (-data[j] * scale)-offset+height);
                cv::line(graph, p1, p2, cv::Scalar(color[2],color[1],color[0]), 1);
            }
        }
        break;
    }
    case points :{
        if (data.size() > 0){
            for(unsigned int i = 0; i < data.size(); ++i){
                p1 = cv::Point2d(i, (-data[i] * scale)-offset+height);
                cv::circle(graph, p1, 0.3, cv::Scalar(color[2],color[1],color[0]), 2);
            }
        }
        break;
    }
    }
    char min [20];
    sprintf (min, "%f", offset);
    cv::putText(graph, min, cv::Point2d(10,height-10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
    char plus [20];
    sprintf (plus, "%f", float(offset+height/scale));
    cv::putText(graph, plus, cv::Point2d(10,15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));

}

void Graph::show(){
    cv::imshow("Graph", graph);
    cv::waitKey(3);
}

void Graph::emptygraph(){
    cv::Mat newgraph(height, width, CV_8UC3, cv::Scalar(100, 100, 100));
    graph = newgraph;
}

