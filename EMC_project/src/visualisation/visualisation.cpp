#include "visualisation.h"

Visualisation::Visualisation(unsigned long width_, unsigned long height_){
    resolution = RESOLUTION;
    width = width_;
    height = height_;
    emptycanvas();
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);
}

void Visualisation::addObject(Object& objectorigional){

    Object obj = objectorigional;
    obj.transform(-origin.x, -origin.y, 0); // TRANSFORM FOR VISUALISATION > IF NO COPY IS MADE TRANSFORM BACK AT THE BOTTOM
    obj.transform(0, 0, -origin.a); // TRANSFORM FOR VISUALISATION > IF NO COPY IS MADE TRANSFORM BACK AT THE BOTTOM
    cv::Point2d p1;
    cv::Point2d p2;
    switch(obj.drawtype){
    case Objectoptions::lines :{
        if (obj.points.size() > 1){
            Vector2 pavg(0,0.2);
            for(unsigned int i = 0; i < obj.points.size()+obj.connection; ++i){
                unsigned int j = (i + 1) % obj.points.size();
                p1 = worldToCanvas(obj.points[i].location);
                p2 = worldToCanvas(obj.points[j].location);
                cv::line(canvas, p1, p2, cv::Scalar(obj.color[2],obj.color[1],obj.color[0]), 1);
            }
            for(unsigned int i = 0; i < obj.points.size(); ++i){
                pavg = pavg + obj.points[i].location;
            }
            pavg = pavg / (obj.points.size()+obj.connection+1);
            cv::Point2d textl = worldToCanvas(pavg);
            //cv::putText(canvas, obj.name, textl, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
//            char angle[20];
//            sprintf(angle, "%4.2f", obj.angle());
//            p1.y = p1.y - 15;
//            cv::putText(canvas, angle, p1, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
//            char length[20];
//            sprintf(length, "%4.2f", obj.length());
//            p1.y = p1.y - 15;
//            cv::putText(canvas, length, p1, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        }
        break;
    }
    case Objectoptions::points :{
        if (obj.points.size() > 0){
            Vector2 pavg(0,0);
            for(unsigned int i = 0; i < obj.points.size(); ++i){
                p1 = worldToCanvas(obj.points[i].location);
                pavg = pavg + obj.points[i].location;
                cv::circle(canvas, p1, obj.pointradius, cv::Scalar(obj.color[2],obj.color[1],obj.color[0]), 2);
            }
            pavg = pavg /obj.points.size();
            cv::Point2d textl = worldToCanvas(pavg);
            cv::putText(canvas, obj.name, textl, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
        }
        break;
    }
    }
    if(obj.type == Object::wall || obj.type == Object::roomwall){
        for(unsigned int i = 0; i < obj.points.size(); ++i){
            Point objp = obj.points[i];
            p1 = worldToCanvas(obj.points[i].location);
            if(objp.property == Point::connected){
                Color col = {255,255,0};
                cv::circle(canvas, p1, obj.pointradius, cv::Scalar(col[2], col[1], col[0]), 2);
            }else if(objp.property == Point::convex){
                Color col = {0,0,255};
                cv::circle(canvas, p1, obj.pointradius, cv::Scalar(col[2], col[1], col[0]), 2);
            }else if(objp.property == Point::concave){
                Color col = {0,255,255};
                cv::circle(canvas, p1, obj.pointradius, cv::Scalar(col[2], col[1], col[0]), 2);
            }else{
                Color col = {255,255,255};
                cv::circle(canvas, p1, obj.pointradius, cv::Scalar(col[2], col[1], col[0]), 2);
            }
        }
    }
    //obj.transform(origin.x, origin.y, origin.a, 0); // TRANSFORM BACK TO ORIGIONAL FRAME
}

void Visualisation::addMap(Map& map){
    for(unsigned long i = 0; i < map.objects.size(); ++i){
        addObject(map.objects[i]);
    }
}

void Visualisation::addSafeDis(close_prx safe, Position T){
    safe.dt_angle;
    safe.start_angle;
    safe.toClose;

    Color color = {0, 255, 0};
    Name name = "";
    Object SafeDis(Object::safeDis, name, color, points, open);

    Color Ncolor = {255, 0, 0};
    Name Nname = "";
    Object NonSafeDis(Object::safeDis, Nname, Ncolor, points, open);

    for(unsigned long i = 0; i < safe.toClose.size(); ++i){
        float a = safe.start_angle + safe.dt_angle*i;
        float x = CLOSEPROXIMITY * cos(a);
        float y = CLOSEPROXIMITY * sin(a);
        if(safe.toClose[i]==false){
            SafeDis.points.push_back(Point(Vector2(x,y),1,Point::floating));
        }else{
            NonSafeDis.points.push_back(Point(Vector2(x,y),1,Point::floating));
        }
    }
    SafeDis.transform(T.x,T.y,T.a);
    NonSafeDis.transform(T.x,T.y,T.a);
    addObject(SafeDis);
    addObject(NonSafeDis);
}

void Visualisation::setorigin(Position origin_){
    origin = origin_;
}

void Visualisation::visualize(){
    cv::imshow("Visualisation", canvas);
    cv::waitKey(3);
}

void Visualisation::emptycanvas(){
    cv::Mat newcanvas(height, width, CV_8UC3, cv::Scalar(100, 100, 100));
    canvas = newcanvas;
}

cv::Point2d Visualisation::worldToCanvas(const Vector2& p){
    return cv::Point2d(-p.y / resolution, -p.x / resolution) + canvas_center;
}
