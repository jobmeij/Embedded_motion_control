#include "perception.h"

////////////////////////////////EVALUATE///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::evaluate(){
    if(PC_DEBUG == true){cout << "Preceptor debuging enabled " << endl;}
    Map currentmap;
    Map& localmap = worldmodel->getlocalmap();
    Map& globalmap = worldmodel->getglobalmap();
    Position& curpos = worldmodel->getcurrentposition();
    Position& zeropos = worldmodel->getzeroposition();
    Block::Modes mode = worldmodel->getmode(Block::Perceptor);
    vector<node>& nodelist = worldmodel->getnodelist();
    vector<link>& linklist = worldmodel->getlinklist();
    vector<node>& path = worldmodel->getpath();
    float& curbestfitscore = worldmodel->getfitscore();

    // Init
    if(mode == Block::MODE_INIT){
        curbestfitscore = 5;
        init();
        creategm(globalmap,nodelist,linklist);
        transform(globalmap,nodelist,path,-5,-3,M_PI/3,false);
        combinepoints(globalmap);
        pointproperties(globalmap, curpos);
        projectpoints(globalmap);
//        identifydoors(globalmap);
//        identifyrooms(globalmap);
        worldmodel->setstatus(Block::Perceptor,Block::STATE_DONE);
    }

    // Fit
    if(mode == Block::PC_MODE_FIT){
//        Position zero = worldmodel->getcurrentposition();
//        worldmodel->setzeroposition(zero.x-2, zero.y+2, zero.a-M_PI/2);

        if (robotIO->readLaserData(laserdatainst)){
            locate();
            closeproximity(laserdatainst);
            scan(currentmap, curpos, laserdatainst);
            merge(currentmap, localmap, curpos); //combine current map and localmap into updated localmap
            pointproperties(localmap, curpos);
//            identifydoors(localmap);
        }


        bool goodfit = fitmap(localmap,globalmap,nodelist, path, curbestfitscore);
        //goodfit = true;   //overrule fitting

        if (goodfit == true){
            cout << "map is fitted" << endl;
            worldmodel->setstatus(Block::Perceptor,Block::STATE_DONE);
        }else{
            worldmodel->setstatus(Block::Perceptor,Block::STATE_BUSY);
        }
    }

    // Execute
    if(mode == Block::MODE_EXECUTE){

        if (robotIO->readLaserData(laserdatainst)){
            locate();
            closeproximity(laserdatainst);
            scan(currentmap, curpos, laserdatainst);
            merge(currentmap, localmap, curpos); //combine current map and localmap into updated localmap
            pointproperties(localmap, curpos);
            align(currentmap, globalmap, localmap, nodelist, path);
//            identifydoors(localmap);
        }


        if(PC_DEBUG == true){cout << endl;}
        worldmodel->setstatus(Block::Perceptor,Block::STATE_DONE);
    }
}

////////////////////////////////INIT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::init(){
    cout << "INIT POSITION" << endl;
    while(!robotIO->readOdometryData(odometrydata)){}
    worldmodel->setzeroposition(odometrydata.x, odometrydata.y, odometrydata.a);
    worldmodel->setcurrentposition(0,0,0);
    cout << "ZERO POSITION > X:" << odometrydata.x << " Y:" << odometrydata.y << " A:" << odometrydata.a << endl;
}

////////////////////////////////CREATE GLOBAL MAP//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::creategm(Map& globalmap, vector<node>& nodelist, vector<link>& linklist){
    // Read the json map
    std::ifstream i("../src/Json_maps/finalmap.json");
    nlohmann::json map = nlohmann::json::parse(i);

//    // read json nodes
//    std::ifstream s("../src/Json_maps/Testing_nodes.json");
//    nlohmann::json nodes = nlohmann::json::parse(s);

    // define map colors and variables
    Color gmwalls = {255, 0, 255};
    Color gmcabinet = {255, 255, 0};
    Color gmstart = {100, 255, 0};
    Color gmnodes = {255, 100, 100};
    Points pointlist;
    Points Nodes;

    for (const auto& p : map.at("points") ){

        assert(p.is_object());
        assert(p.find("x") != p.end());  // check for key x
        assert(p.find("y") != p.end());  // check for key y
        Point test(Vector2(p["x"],p["y"]),1,Point::floating);
        pointlist.push_back(test);
    }

    // conections between points
    unsigned int w = 0;
    for (const auto& l : map.at("walls") ){
        char name[15] = "";
        sprintf(name,"W:%u",w);
        w++;
        Object wall(Object::wall,name,gmwalls,lines,open);
        wall.points.push_back(pointlist[l[0]]);
        wall.points.push_back(pointlist[l[1]]);
        //wall.points[0].parents.push_back(&wall);
        //wall.points[1].parents.push_back(&wall);
        globalmap.objects.push_back(wall);
    }

    // Connection of the cabinets:
    unsigned int c = 0;
    for (const auto& cab : map.at("cabinets")){
        char name[15] = "";
        sprintf(name,"C:%u",c);
        c++;
        Object cabinet(Object::cabinet,name,gmcabinet,lines,open);
        for (const auto& l : cab){
            cabinet.points.push_back(pointlist[l[0]]);
            cabinet.points.push_back(pointlist[l[1]]);
            //cabinet.points[0].parents.push_back(&cabinet);
            //cabinet.points[1].parents.push_back(&cabinet);
        }
        globalmap.objects.push_back(cabinet);
    }

    for (const auto& str : map.at("start")){
        char name[15] = "start";

        Object start(Object::start,name,gmstart,lines,open);
        for (const auto& l : str){
            start.points.push_back(pointlist[l[0]]);
            start.points.push_back(pointlist[l[1]]);
            //cabinet.points[0].parents.push_back(&cabinet);
            //cabinet.points[1].parents.push_back(&cabinet);
        }
        globalmap.objects.push_back(start);
    }

    unsigned long cabinet = 0;
    for (const auto& p : map.at("nodes") ){
        assert(p.is_object());
        assert(p.find("x") != p.end());  // check for key x
        assert(p.find("y") != p.end());  // check for key y
        assert(p.find("a") != p.end());  // check for key a
        if(cabinet < CABINETNUMBERS){
            nodelist.push_back(node(Vector2(p["x"],p["y"]),float(p["a"])*2*M_PI,node::destination));
        }else{
            nodelist.push_back(node(Vector2(p["x"],p["y"]),float(p["a"])*2*M_PI,node::normal));
        }
        cabinet++;
    }

    for (const auto& l : map.at("nodelinks")){
        linklist.push_back(link(&nodelist[l[0]],&nodelist[l[1]],false));
    }

    int roompoints[] = {16,17,18,19,20,21,22,23};
    for(unsigned long i = 1; i < sizeof(roompoints)/sizeof(*roompoints); i++){
        char name[15] = "RW";
        Color gmstartroom = {0, 0, 0};
        Object roomwall(Object::roomwall,name,gmstartroom,lines,open);
        int point1 = roompoints[i-1];
        int point2 = roompoints[i];
        roomwall.points.push_back(pointlist[point1]);
        roomwall.points.push_back(pointlist[point2]);
        globalmap.objects.push_back(roomwall);
    }

    int roomcabpoints[] = {52,53,54,55,52};
    for(unsigned long i = 1; i < sizeof(roomcabpoints)/sizeof(*roomcabpoints); i++){
        char name[15] = "RW";
        Color gmstartroom = {0, 0, 0};
        Object roomwall(Object::roomwall,name,gmstartroom,lines,open);
        int point1 = roomcabpoints[i-1];
        int point2 = roomcabpoints[i];
        roomwall.points.push_back(pointlist[point1]);
        roomwall.points.push_back(pointlist[point2]);
        globalmap.objects.push_back(roomwall);
    }
}
////////////////////////////////FITMAP FUNCTION////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Perception::fitmap(Map &localmap,Map &globalmap, vector<node>& nodelist, vector<node>& path, float& curbestfitscore){
    bool goodfit = false;

    // match all local walls with global wallrooms

    Object::types globaltype = Object::roomwall; //roomwall or wall both possible

    float bestfitscore =-1;
    unsigned long bestnumberofmatchingwalls = 0;
    int bestanglesign = 1;
    float bestlocAngle = 0;
    float bestglobAngle = 0;
    Vector2 bestlocalp;
    Vector2 bestglobalp;

    for(unsigned long i1 = 0; i1 < localmap.objects.size(); i1++){
        for(unsigned long i2 = 0; i2 < globalmap.objects.size(); i2++){
            Object& obj1 = localmap.objects[i1];
            Object& obj2 = globalmap.objects[i2];

            if(obj1.type == Object::wall && obj2.type == globaltype && obj1.points.size() == 2 && obj2.points.size() == 2){

                //transform variables
                float locAngle = 0;
                float globAngle = 0;
                Vector2 localp;
                Vector2 globalp;

                //Transform so 2 lines are on top of each other
                locAngle = obj1.angle();
                globAngle = obj2.angle();
                for(int t = 0; t < 2; t++){
                    int s = 1;
                    if(t == 1){s = -1;}
                    globalmap.transform(0,0,s*(locAngle-globAngle));
                    localp = obj1.middle();
                    globalp = obj2.middle();
                    globalmap.transform(localp.x-globalp.x,localp.y-globalp.y,0);

                    //Check the fit score
                    float fitscore = 0;
                    float fitlength = 0;
                    unsigned long numberofmatchingwalls = 0;
                    unsigned long numberofprojectedwalls = 0;
                    for(unsigned long i1 = 0; i1 < localmap.objects.size(); i1++){
                        for(unsigned long i2 = 0; i2 < globalmap.objects.size(); i2++){
                            Object& obj1 = localmap.objects[i1];
                            Object& obj2 = globalmap.objects[i2];

                            if(obj1.type == Object::wall && obj2.type == globaltype && obj1.points.size() == 2 && obj2.points.size() == 2){
                                float avgdist = obj1.middle().distance(obj2.middle());
                                float angle = obj1.smallestrelativeangle(obj2);
                                float pdist = obj1.averageperpendiculardistance(obj2);

                                bool avgcloseto = avgdist < FITDIFFERENCE;
                                bool parallel = angle < ANGLEMERGEDIFFERENCE*0.5;
                                bool pcloseto = pdist < DISTMERGEDIFFERENCE*0.5;

//                                cout << "avgcloseto: " << avgcloseto << endl;
//                                cout << "parallel: " << parallel << endl;
//                                cout << "pcloseto: " << pcloseto << endl;
                                    // match two perfect lines and skip transormed line
                                    if(parallel && pcloseto && (avgdist > 0.0001)){
                                        if(avgcloseto){
                                        fitscore += avgdist;
                                        fitlength += obj1.length();
                                        numberofmatchingwalls ++;
                                        }
                                    // match a projection of a line
                                        else{
                                            for(unsigned long pnt1 = 0; pnt1 < obj2.points.size(); pnt1++){
                                                    Point& pointobj2 = obj2.points[pnt1];
                                                    Point& point2obj2 = obj2.points[(pnt1+1)%2];
                                                    Point& point1obj1 = obj1.points[0];
                                                    Point& point2obj1 = obj1.points[1];
                                                    float dist = pointobj2.location.distance(point1obj1.location);
                                                    float dist2 = pointobj2.location.distance(point2obj1.location);
                                                    float ldiff =  obj2.length()-obj1.length();

                                                    // check if the walls have one common corner
                                                    if((dist < FITDIFFERENCE || dist2 < FITDIFFERENCE) && ldiff> 0){
    //                                                        cout << "global match point: " << pointobj2.location.x << "y: "<< pointobj2.location.y << endl;
    //                                                        cout << "global other point: " << point2obj2.location.x << "y: "<< point2obj2.location.y << endl;
    //                                                        cout << "local point 1: " << point1obj1.location.x << "y: "<< point1obj1.location.y << endl;
    //                                                        cout << "local point 2: " << point2obj1.location.x << "y: "<< point2obj1.location.y << endl;
    //                                                        cout << "angle2"<< obj2.angle() << endl;
    //                                                        cout << "angle1"<< obj1.angle() << endl;

                                                            float angleobj2 = obj2.angle();
                                                            Vector2 xydis(ldiff*cos(angleobj2),ldiff*sin(angleobj2));
                                                            point2obj2.location = point2obj2.location-xydis;
                                                            if (obj2.length()>obj1.length()+0.1){
                                                                  point2obj2.location = point2obj2.location+(xydis + xydis);
                                                                  xydis = xydis*-1;
                                                            }
                                                            avgdist = obj1.middle().distance(obj2.middle());

    //                                                        cout << "angle2"<< obj2.angle() << endl;
    //                                                        cout << "angle1"<< obj1.angle() << endl;
    //                                                        cout << "avgdist: "<< avgdist << endl;
    //                                                        cout << "xydis: " << xydis.x << "y: "<< xydis.y << endl;
    //                                                        cout << "adiff: "<< adiff << endl;
    //                                                        cout << "length obj2: " << obj2.length() << endl;
    //                                                        cout << "length obj1: " << obj1.length() << endl;
    //                                                        cout << "length ldiff " << ldiff << endl;

                                                            point2obj2.location = point2obj2.location+xydis;

                                                            if (avgdist < FITDIFFERENCE){
                                                                fitscore += avgdist;
                                                                fitlength += obj1.length();
                                                                numberofprojectedwalls ++;
                                                            }

                                                    }
                                            }
                                        }
                                    }


                                if((numberofmatchingwalls + numberofprojectedwalls )> bestnumberofmatchingwalls){
//                                    cout << "best number of matching walls: " << bestnumberofmatchingwalls << endl;
//                                    cout << "number of projected walls: " << numberofprojectedwalls << endl;
                                    bestglobalp = globalp;
                                    bestglobAngle = globAngle;
                                    bestlocalp = localp;
                                    bestlocAngle = locAngle;

                                    bestnumberofmatchingwalls = numberofmatchingwalls+numberofprojectedwalls;
                                    bestfitscore = fitscore;
                                    bestanglesign = s;
                                }
                            }
                        }
                    }

                    // Transform back
                    globalmap.transform(-(localp.x-globalp.x),-(localp.y-globalp.y),0);
                    globalmap.transform(0,0,s*-(locAngle-globAngle));
                }
            }
        }
    }

    if(bestnumberofmatchingwalls > 4 && bestfitscore < FITDIFFERENCE*bestnumberofmatchingwalls){
        cout << "good fit" << endl;
        cout << "n walls: "<< bestnumberofmatchingwalls << endl;
        cout << "fitscore: "<< bestfitscore << endl;

        curbestfitscore = bestfitscore/2;
        transform(globalmap, nodelist,path, bestlocalp.x-bestglobalp.x, bestlocalp.y-bestglobalp.y, bestlocAngle-bestglobAngle,true);
        goodfit = true;
    }
    else if(bestnumberofmatchingwalls > 2 && bestfitscore < curbestfitscore*bestnumberofmatchingwalls){
        cout << "best fit up to now" << endl;
        cout << "n walls: "<<bestnumberofmatchingwalls << endl;
        cout << "fitscore: "<< bestfitscore << endl;

        curbestfitscore = bestfitscore;
        transform(globalmap, nodelist,path, bestlocalp.x-bestglobalp.x, bestlocalp.y-bestglobalp.y, bestlocAngle-bestglobAngle,true);
    }

    return goodfit;
}
////////////////////////////////ALIGN FUNCTION////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::align(Map& scanmap, Map& globalmap, Map& localmap, vector<node>& nodelist, vector<node>& path){

    Vector2 avgoffset(0,0);
    float avganglediff = 0;
    unsigned long nmatchingwalls = 0;
    for(unsigned long i1 = 0; i1 < scanmap.objects.size(); i1++){
        for(unsigned long i2 = 0; i2 < globalmap.objects.size(); i2++){
            Object& obj1 = scanmap.objects[i1];
            Object& obj2 = globalmap.objects[i2];

            if(obj1.type == Object::wall && obj2.type == Object::wall && obj1.points.size() == 2 && obj2.points.size() == 2){
                Vector2 diff = obj1.middle() - obj2.middle();
                float avgdist = obj1.middle().distance(obj2.middle());

                float angle = obj1.smallestrelativeangle(obj2);
                float pdist = obj1.averageperpendiculardistance(obj2);

                bool avgcloseto = avgdist < DISTANCEDIFFERENCE;
                bool parallel = angle < ANGLEMERGEDIFFERENCE;
                bool pcloseto = pdist < DISTMERGEDIFFERENCE;

                float scanangle = obj1.angle();
                float globalangle = obj2.angle();

                if(parallel && avgcloseto && pcloseto){
                    avgoffset = avgoffset + diff;
                    avganglediff += (scanangle - globalangle);
                    nmatchingwalls ++;
                }
            }
        }
    }

    avgoffset = avgoffset/(float)nmatchingwalls;
    avganglediff = avganglediff/(float)nmatchingwalls;
    float shiftrate = 0.01;
    if(nmatchingwalls > 2 && avgoffset.length() < DISTANCEDIFFERENCE && avganglediff < ANGLEMERGEDIFFERENCE){
        transform(globalmap, nodelist, path, avgoffset.x*shiftrate, avgoffset.y*shiftrate, -avganglediff*shiftrate,true);
        localmap.transform(avgoffset.x*shiftrate, avgoffset.y*shiftrate, -avganglediff*shiftrate);
    }
}

////////////////////////////////LOCATE FUNCTION////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::locate(){
    if (robotIO->readOdometryData(odometrydata)){
        Position zero = worldmodel->getzeroposition();
        Vector2 ododata(-(odometrydata.y-zero.y), (odometrydata.x-zero.x));
        ododata.transform(0,0,-zero.a-(M_PI/2));
        worldmodel->setcurrentposition(ododata.x, ododata.y, odometrydata.a-zero.a);
    }
}
////////////////////////////////CLOSE PROXIMITY////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::closeproximity(emc::LaserData laserdata){
    // if no laserdata is availible still create object but set property valid to false
    close_prx safeDis;
        vector<float> ranges = laserdata.ranges;
        vector<bool> toClose;
        vector<float> closeDist;
        double angleStart = laserdata.angle_min;
        double angleDt = laserdata.angle_increment;
        unsigned int angleOffset = 10; //number of samples to excluded
        unsigned int closeDt = 20; //steps in between
        unsigned int laserN = ranges.size();
//        close_prx safeDis;

        safeDis.start_angle = angleStart+(angleOffset*angleDt);
        safeDis.dt_angle = closeDt*angleDt;
//        std::cout << "set start angle at: " << safeDis.dt_angle << std::endl;

        vector<float>::const_iterator begin = ranges.begin();
        vector<float>::const_iterator last = ranges.begin() + ranges.size();
        vector<float> rangesUpdate(begin+angleOffset, last-angleOffset);

        laserN = (rangesUpdate.size())/closeDt+1;
        for (unsigned int i = 0; i < laserN; i++){

            float datapoint = rangesUpdate[i*closeDt];
            closeDist.push_back(datapoint);

            if (datapoint < CLOSEPROXIMITY && datapoint > 0.1){
                toClose.push_back(true);
//                cout << "to close to a wall, angle: " <<  ((angleStart+i*safeDis.dt_angle)*360)/(2*M_PI)  << endl;

            }else{
                toClose.push_back(false);
            }
        }
        safeDis.toClose = toClose;
        safeDis.distances = closeDist;
        safeDis.valid = true;
        worldmodel->setClosePrx(safeDis);
//      To obtain the data sent to the world model:
//      close_prx varname = worldmodel->getToClose();
//    else{
//        safeDis.valid = false;
//        worldmodel->setClosePrx(safeDis);
//    }
}

////////////////////////////////SCAN FUNCTION//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::scan(Map &currentmap, Position& curpos, emc::LaserData laserdata){
    std::vector<Vector2> cartesiandata;
    std::vector<float> laserranges = laserdata.ranges;

    polar2cartesian(cartesiandata, laserranges, laserdata.angle_min, laserdata.angle_increment);
    resample(cartesiandata, SAMPLEDISTANCE);
    fitlines(cartesiandata, currentmap);

    //Only used for visualising points in clusters
    Color color2 = {255,0,0};
    Name name2 = "";
    Object wallpoints(Object::test,name2,color2,points,open);
    wallpoints.pointradius = 0.3;
    for(unsigned long i = 0; i < cartesiandata.size(); ++i){
        Vector2 POS = cartesiandata[i];
        POS.transform(curpos.x, curpos.y, curpos.a);
        Point currentpoint(POS,1,Point::floating);
        wallpoints.points.push_back(currentpoint);
    }
    visualisation->addObject(wallpoints);

}

////////////////////////////////MERGE FUNCTION/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::merge(Map &currentmap, Map &localmap, Position &curpos){

    currentmap.transform(curpos.x, curpos.y, curpos.a);
    worldmodel->setcurrentmap(currentmap);

    //add currentmap to localmap
    Color localmapcolor = {0,255,0};
    for(unsigned long ci = 0; ci < currentmap.objects.size(); ci++){
        if(localmap.objects.size() <= MAXMAPSIZE){//ensure memory doenst blow up if the removing of objects in the local map doesn work properly
            Object obj = currentmap.objects[ci];
            obj.color[0] = localmapcolor[0];
            obj.color[1] = localmapcolor[1];
            obj.color[2] = localmapcolor[2];
            localmap.objects.push_back(obj);
        }else{
            cout << "MAP FULL" << endl;
        }
    }

    if(localmap.objects.size() > 0){

        //mark similar walls
        for(unsigned long i1 = 0; i1 < localmap.objects.size()-1; i1++){ //one way check > [0-1 | 0-2 | 0-3 | 1-2 | 1-3 | 2-3]
            for(unsigned long i2 = i1+1; i2 < localmap.objects.size(); i2++){
                Object& obj1 = localmap.objects[i1];
                Object& obj2 = localmap.objects[i2];
                if(obj1.type == Object::wall && obj2.type == Object::wall && obj1.points.size() == 2 && obj2.points.size() == 2){
                    float angle = obj1.smallestrelativeangle(obj2);
                    float pdist = obj1.averageperpendiculardistance(obj2);
                    float gdist = obj1.gapdistance(obj2);
                    bool parallel = angle < ANGLEMERGEDIFFERENCE;
                    bool pcloseto = pdist < DISTMERGEDIFFERENCE;
                    bool gcloseto = gdist < DISTMERGEDIFFERENCE;
                    //cout << "i1:" << i1 << " i2:" << i2 << " " << parallel << pcloseto << gcloseto << obj1.remove << obj2.remove << endl;
                    if(parallel && pcloseto && gcloseto ){//&& (obj1.remove == false && obj2.remove == false)){ //&& (obj1.newobject || obj2.newobject)){
                        //cout << "Similar walls found" << endl;
                        float maxdist = 0;
                        Point point1M;
                        Point point2M;
                        float l1 = obj1.length();
                        float l2 = obj2.length();
                        string chosencase = "";
                        for(unsigned int p1 = 0; p1 < obj1.points.size(); p1++){
                            for(unsigned int p2 = 0; p2 < obj2.points.size(); p2++){
                                Point& pointobj1 = obj1.points[p1];
                                Point& pointobj2 = obj2.points[p2];
                                Point& otherpointobj1 = obj1.points[p1+1%1];
                                Point& otherpointobj2 = obj2.points[p2+1%1];
                                float dist = pointobj1.location.distance(pointobj2.location);
                                float otherdist = otherpointobj1.location.distance(otherpointobj2.location);
                                if(dist > maxdist && dist > l1 && dist > l2){
                                    point1M = pointobj1;
                                    point2M = pointobj2;
                                    maxdist = dist;
                                    chosencase = "Extended";
                                }
                                if(l1 >= maxdist){
                                    point1M = obj1.points[0];
                                    point2M = obj1.points[1];
                                    maxdist = l1;
                                    chosencase = "Line 1";
                                }
                                if(l2 >= maxdist){
                                    point1M = obj2.points[0];
                                    point2M = obj2.points[1];
                                    maxdist = l2;
                                    chosencase = "line 2";
                                }
                            }
                        }
                        //cout << "Chosen case: " << chosencase << endl;
                        obj1.points[0] = point1M;
                        obj1.points[1] = point2M;
                        obj2.remove = true;
                    }
                }
            }
        }
        localmap.removeobjects(); //remove all objects with remove flag

        removepointproperties(localmap, Object::wall);

        combinepoints(localmap);

        //remove floating or small walls
        for(unsigned long li = 0; li < localmap.objects.size(); li++){
            Object& obj = localmap.objects[li];
            if(obj.type == Object::wall && obj.points.size() == 2){
                if(obj.points[0].property == Point::floating && obj.points[1].property == Point::floating){
                    obj.remove = true;
                }
                if(obj.length() < DISTMERGEDIFFERENCE){
                    obj.remove = true;
                }
            }
        }

        localmap.removeobjects(); //remove all objects with remove flag
        localmap.setobjectsold(); //set all new objects to old objects

    }
}

////////////////////////////////IDENTIFY DOORS//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::identifydoors(Map &map){

    if(map.objects.size() > 0){

        //Identify doors
        for(unsigned long i1 = 0; i1 < map.objects.size()-1; i1++){ //one way check > [0-1 | 0-2 | 0-3 | 1-2 | 1-3 | 2-3]
            for(unsigned long i2 = i1+1; i2 < map.objects.size(); i2++){
                Object& obj1 = map.objects[i1];
                Object& obj2 = map.objects[i2];
                if(obj1.type == Object::wall && obj2.type == Object::wall){
                    for(unsigned long p1 = 0; p1 < obj1.points.size(); p1++){
                        for(unsigned long p2 = 0; p2 < obj2.points.size(); p2++){
                            Point* point1 = &obj1.points[p1];
                            Point* point2 = &obj2.points[p2];
                            //Point* doorpoint = NULL;
                            bool sameparent = point1->sameparentobject(*point2);

                            float gapdist = obj1.gapdistance(obj2);
                            float wallangle = obj1.smallestrelativeangle(obj2);
                            float pdist = obj1.averageperpendiculardistance(obj2);

                            bool convexconvex = (point1->property == Point::convex && point2->property == Point::convex);
                            bool convexfloat = ((point1->property == Point::convex && point2->property == Point::floating)
                                             || (point1->property == Point::floating && point2->property == Point::convex));
                            bool floatfloat = (point1->property == Point::floating && point2->property == Point::floating);
                            bool projection = ((point1->projectionpoint != NULL && point1->property == Point::convex));

                            if( (convexconvex || projection) && !sameparent){

                                Color doorcolor = {0, 0, 255};
                                Name name = "D";
                                Object door(Object::door, name, doorcolor, Objectoptions::lines, Objectoptions::open);
                                if(convexconvex){
                                    door.points.push_back(*point1);
                                    door.points.push_back(*point2);
                                }else if(point2->projectionpoint != NULL){
                                    point1 = point2->projectionpoint;
                                    door.points.push_back(*point1);
                                    door.points.push_back(*point2);
                                }else if(point1->projectionpoint != NULL){
                                    point2 = point1->projectionpoint;
                                    door.points.push_back(*point1);
                                    door.points.push_back(*point2);
                                }
                                door.remove = true;
                                float paralleldoor1 = obj1.smallestrelativeangle(door);
                                float paralleldoor2 = obj2.smallestrelativeangle(door);
                                bool parallel = wallangle < ANGLEMERGEDIFFERENCE && paralleldoor1 < ANGLEMERGEDIFFERENCE && paralleldoor2 < ANGLEMERGEDIFFERENCE;

                                if(door.length() > MINDOORLENGTH && door.length() < MAXDOORLENGTH){
                                    bool door1 = !projection && convexconvex && door.length() == gapdist && parallel && pdist < DISTMERGEDIFFERENCE;
                                    bool door2 = !convexconvex && projection && abs(M_PI/2 - wallangle) < ANGLEMERGEDIFFERENCE;
                                    if(door1 || door2){
                                        map.objects.push_back(door);
                                        Object& doorobj = map.objects.back();
                                        doorobj.points[0].parents.push_back(&doorobj);
                                        doorobj.points[1].parents.push_back(&doorobj);
                                        point1->parents.push_back(&doorobj);
                                        point2->parents.push_back(&doorobj);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

////////////////////////////////IDENTIFY ROOMS//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::identifyrooms(Map &map){

     if(map.objects.size() > 0){

//         //map.printparents();

//        //Identify rooms
//        for(unsigned long io = 0; io < map.objects.size(); io++){
//            Object* obj = &map.objects[io];
//            if(obj->type == Object::wall){

//                Color roomcolor = {255, 255, 255};
//                Name name = "R";
//                Object room(Object::room, name, roomcolor, Objectoptions::lines, Objectoptions::closed);

//                vector<Object*> roomobjects;
//                roomobjects.clear();
//                roomobjects.push_back(obj);

//                bool done = false;
//                int c = 0;
//                while(!done && c < MAXMAPSIZE){
//                    done = true;

//                    bool sameroom = false;
//                    int samepoints = 0;
//                    for(unsigned long ri = 0; ri < map.objects.size(); ri++){
//                        Object& roomobj = map.objects[ri];
//                        if(roomobj.type == Object::room && c > 1){
//                            for(unsigned long ip1 = 0; ip1 < roomobj.points.size(); ip1++){
//                                for(unsigned long ip2 = 0; ip2 < room.points.size(); ip2++){
//                                    Point &p1 = roomobj.points[ip1];
//                                    Point &p2 = room.points[ip2];
//                                    if(p1.location == p2.location){
//                                        samepoints ++;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                    if(samepoints > 2){
//                        sameroom = true;
//                    }

//                    if(!sameroom && obj->points.size() > 1){
//                        Point& curpoint = obj->points[0];
//                        room.points.push_back(curpoint);

//                        Point* nextpoint = NULL;
//                        if(obj->projectedpoints.size() > 0){
//                            unsigned long bestpointindex = 0;
//                            float bestdist = obj->length();
//                            for(unsigned long ip = 0; ip < obj->projectedpoints.size(); ip++){
//                                Point p = *obj->projectedpoints[ip];
//                                float dist = p.location.distance(curpoint.location);
//                                if(dist < bestdist){
//                                    bestdist = dist;
//                                    bestpointindex = ip;
//                                }
//                            }
//                            nextpoint = obj->projectedpoints[bestpointindex];
//                        }else{
//                            nextpoint = &obj->points[1];
//                        }
//                        if(nextpoint->parents.size() > 1){

//                            Object* temp = NULL;
//                            float objangle = obj->anglefrom(curpoint);
//                            float bestangle = 2*M_PI;
//                            float bestangle2 = 0;

//                            for(unsigned long pa = 0; pa < nextpoint->parents.size(); pa++){
//                                Object* nextobj = nextpoint->parents[pa];
//                                float nextobjangle = nextobj->anglefrom(*nextpoint);
//                                if(nextobj != obj){
//                                    if(abs(nextobjangle - objangle) < bestangle && nextobjangle <= objangle){
//                                        bestangle = abs(nextobjangle - objangle);
//                                        temp = nextobj;
//                                    }
//                                    if(temp == NULL && abs(nextobjangle - objangle) > bestangle2 && nextobjangle > objangle){
//                                        bestangle2 = abs(nextobjangle - objangle);
//                                        temp = nextobj;
//                                    }
//                                }
//                            }
//                            obj = temp;

//                            if(obj != NULL){
//                                bool sameobject = false;
//                                for(unsigned long ro = 0; ro < roomobjects.size(); ro++){
//                                    if(roomobjects[ro] == obj){
//                                        sameobject = true;
//                                    }
//                                }

//                                if(sameobject && c > 3){
//                                    map.objects.push_back(room);
////                                    Object& roomobj = map.objects.back();
////                                    for(unsigned long rp = 0; rp < roompoints.size(); rp++){
////                                        Point* roompoint = roompoints[rp];
////                                        roompoint->parents.push_back(&roomobj);
////                                    }
//                                }else{
//                                    roomobjects.push_back(obj);
//                                    done = false;
//                                }
//                            }
//                        }
//                    }
//                    c++;
//                }

////                for(unsigned long ob = 0; ob < map.objects.size(); ob++){
////                    Object& obj = map.objects[ob];
////                    if(obj.type == Object::room){
////                        for(unsigned long rp = 0; rp < obj.points.size(); rp++){
////                            Point& roompoint = obj.points[rp];
////                            bool exists = false;
////                            for(unsigned long pa = 0; pa < roompoint.parents.size(); pa++){
////                                if(roompoint.parents[pa] == &obj){
////                                    exists = true;
////                                }
////                            }
////                            if(!exists){
////                                roompoint.parents.push_back(&obj);
////                            }

////                        }
////                    }
////                }

////                break;
//            }
//        }
////        map.printparents();
    }
}

//////////////////////////////////FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Perception::polar2cartesian(std::vector<Vector2> &cartesiandata, std::vector<float> &radii, float angle0, float angleincrement){
    for(unsigned long i = 0; i < radii.size(); ++i){
        float a = angle0 + angleincrement*i;
        if(a > 0 - LASERANGLE/2 && a < LASERANGLE/2 && radii[i] > MINRANGE){
            cartesiandata.push_back(Vector2(radii[i] * cos(a),radii[i] * sin(a)));
        }
    }
}

void Perception::resample(std::vector<Vector2> &cartesiandata, float mindist){
    if(cartesiandata.size() > 0){
        std::vector<Vector2> newdata;
        unsigned long prevpoint = 0;
        newdata.push_back(cartesiandata[0]);
        float dist = 0;
        for(unsigned long i = 0; i < cartesiandata.size(); ++i){
            if(dist > mindist){
                newdata.push_back(cartesiandata[i]);
                prevpoint = i;
                dist = 0;
            }else{
                dist = cartesiandata[i].distance(cartesiandata[prevpoint]);
            }
        }
        cartesiandata = newdata;
    }
}

void Perception::fitlines(std::vector<Vector2> &cartesiandata, Map &map){
    if (cartesiandata.size() > CLUSTERSIZE){
        std::vector<Point> points;
        for(unsigned long i = 0; i < cartesiandata.size(); ++i){
            Point p(cartesiandata[i],0,Point::floating);
            points.push_back(p);
        }

        char name[15] = "W";
        Color wallcolor = {255, 0, 255};
        Object wall(Object::wall,name,wallcolor,lines,open);
        wall.pointradius = 2;
        wall.points.push_back(points[0]);
        wall.points.push_back(points[1]);
        wall.points[0].parents.push_back(&wall);
        wall.points[1].parents.push_back(&wall);

        unsigned long counter = 0;

        for(unsigned long i = 1; i < points.size(); i++){

            float totalfiterror = 0;
            float maxfiterror = 0;
            unsigned long npoints = i - counter + 1;

            wall.points[1] = points[i];
            wall.points[1].parents[0] = &wall;

            for(unsigned long j = counter; j < i; j++){
                Vector2 diff = points[j].location - wall.projection(points[j]);
                totalfiterror += diff.length();
                if(diff.length() > maxfiterror){
                    maxfiterror = diff.length();
                }
            }

            float pointdist = points[i-1].location.distance(points[i].location);
            float fiterror = totalfiterror/npoints;

            if(fiterror > AVGLINEFITERROR || pointdist > DISTANCEDIFFERENCE || i == points.size()-1 || maxfiterror > MAXLINEFITERROR){

                if(fiterror > AVGLINEFITERROR || maxfiterror > MAXLINEFITERROR || pointdist > DISTANCEDIFFERENCE){
                    wall.points[1] = points[i-1];
                    wall.points[1].parents[0] = &wall;
                }

                if(npoints > CLUSTERSIZE){
                    map.objects.push_back(wall);
                }

                if (i < points.size()-1){
                    Object newwall(Object::wall,name,wallcolor,lines,open);
                    newwall.pointradius = 2;
                    newwall.points.push_back(points[i]);
                    newwall.points.push_back(points[i+1]);
                    newwall.points[0].parents.push_back(&newwall);
                    newwall.points[1].parents.push_back(&newwall);

                    wall = newwall;
                }

                counter = i;
            }
        }
    }
}

void Perception::combinepoints(Map &map){

    //Combine close points set property to connected
    for(unsigned long i1 = 0; i1 < map.objects.size()-1; i1++){ //one way check > [0-1 | 0-2 | 0-3 | 1-2 | 1-3 | 2-3]
        for(unsigned long i2 = i1+1; i2 < map.objects.size(); i2++){
            Object& obj1 = map.objects[i1];
            Object& obj2 = map.objects[i2];
            if((obj1.type == Object::wall && obj2.type == Object::wall || obj1.type == Object::roomwall && obj2.type == Object::roomwall) && obj1.points.size() == 2 && obj2.points.size() == 2){
                for(unsigned long p1 = 0; p1 < obj1.points.size(); p1++){
                    for(unsigned long p2 = 0; p2 < obj2.points.size(); p2++){
                        Point& point1 = obj1.points[p1];
                        Point& point2 = obj2.points[p2];
                        float distance = point1.location.distance(point2.location);
                        if(distance < POINTCOMBINEDISTANCE){
                            Point newpoint(Vector2(0,0),0,Point::connected);
                            if(!obj1.newobject && obj2.newobject){
                                newpoint.location = point1.location*0.95 + point2.location*0.05;
                            }else if(!obj2.newobject && obj1.newobject){
                                newpoint.location = point2.location*0.95 + point1.location*0.05;
                            }else{
                                newpoint.location = (point1.location + point2.location)/2;
                            }
                            point1.location = newpoint.location;
                            point1.property = Point::connected;
                            point2.location = newpoint.location;
                            point2.property = Point::connected;
                            point1.connectedwith.push_back(&point2);
                            point2.connectedwith.push_back(&point1);
                            point1.parents.push_back(&obj1);
                            point1.parents.push_back(&obj2);
                            point2.parents.push_back(&obj1);
                            point2.parents.push_back(&obj2);
                        }
                    }
                }
            }
        }
    }
}

void Perception::projectpoints(Map &map){

    //project points on walls
    char name[15] = "P";
    Color projectionscolor = {200,200,200};
    Object projections(Object::projections,name,projectionscolor,points,open);
    projections.pointradius = 4;
    projections.remove = true;
    for(unsigned long i1 = 0; i1 < map.objects.size(); i1++){
        Object& obj1 = map.objects[i1];
        obj1.projectedpoints.clear();
        for(unsigned long i2 = 0; i2 < map.objects.size(); i2++){
            Object& obj2 = map.objects[i2];
            if(obj1.type == Object::wall && obj2.type == Object::wall && obj1.points.size() == 2 && obj2.points.size() == 2 && i1 != i2){
                Point& point1 = obj1.points[0];
                Point& point2 = obj1.points[1];
                Point* point;
                float angle = obj1.smallestrelativeangle(obj2);
                if(abs(M_PI/2-angle) < ANGLEMERGEDIFFERENCE){
                    Vector2 PV1 = obj2.constrainedprojection(point1,MINDOORLENGTH);
                    Vector2 PV2 = obj2.constrainedprojection(point2,MINDOORLENGTH); //retuns (0,0) if point is not projected on the line
                    Vector2 PV(0,0);
                    (PV1.distance(point1.location) <= PV2.distance(point2.location)) ? (PV = PV1, point = &point1) : (PV = PV2, point = &point2);
                    bool PVdist = PV.distance(point->location) > MINDOORLENGTH && PV.distance(point->location) < MAXDOORLENGTH;
                    if(PVdist && !obj1.connectedto(obj2) && PV.length() > 0 && point->property == Point::convex){
                        Point Projectionpoint(PV,0,Point::floating);
                        Projectionpoint.projectionpoint = point;
                        Projectionpoint.parents.push_back(&obj1);
                        projections.points.push_back(Projectionpoint);
                    }
                }
            }
        }
    }
    map.objects.push_back(projections);

    //reference projection points back to wall points
    for(unsigned long i1 = 0; i1 < map.objects.size(); i1++){
        Object& obj1 = map.objects[i1];
        if(obj1.type == Object::projections){
            for(unsigned long p1 = 0; p1 < obj1.points.size(); p1++){
                Point& point1 = obj1.points[p1];
                Point& point2 = *point1.projectionpoint;
                point2.projectionpoint = &point1;
                for(unsigned long pa = 0; pa < point1.parents.size(); pa++){
                    Object& parent = *point1.parents[pa];
                    parent.projectedpoints.push_back(&point1);
                }
            }
        }
    }
}

void Perception::pointproperties(Map &map, Position &curpos){

    //Identify point properties
    for(unsigned long i1 = 0; i1 < map.objects.size()-1; i1++){ //one way check > [0-1 | 0-2 | 0-3 | 1-2 | 1-3 | 2-3]
        for(unsigned long i2 = i1+1; i2 < map.objects.size(); i2++){
            Object& obj1 = map.objects[i1];
            Object& obj2 = map.objects[i2];
            if((obj1.type == Object::wall && obj2.type == Object::wall|| obj1.type == Object::roomwall && obj2.type == Object::roomwall) && obj1.points.size() == 2 && obj2.points.size() == 2){
                for(unsigned long p1 = 0; p1 < obj1.points.size(); p1++){
                    for(unsigned long p2 = 0; p2 < obj2.points.size(); p2++){
                        Point& point1 = obj1.points[p1];
                        Point& point2 = obj2.points[p2];
                        if(point1.connectedpoint(point2)){
                            Color testcolor = {255, 0, 255};
                            Name name = "test";
                            Object test(Object::test, name, testcolor, Objectoptions::lines, Objectoptions::open);
                            test.points.push_back(Point(point1.location,0,Point::connected));
                            test.points.push_back(Point(Vector2(curpos.x, curpos.y),0,Point::connected));
                            test.remove = true;
                            //map.objects.push_back(test);

                            float relativeangle1 = obj1.anglebetween(test);
                            float relativeangle2 = obj2.anglebetween(test);
                            float wallangle = obj1.smallestrelativeangle(obj2);

                            if((2*M_PI - relativeangle1) + relativeangle2 > 2*M_PI){
                                relativeangle2 = 2*M_PI - relativeangle2;
                            }else{
                                relativeangle1 = 2*M_PI - relativeangle1;
                            }

                            if(abs(wallangle-M_PI/2) < ANGLEMERGEDIFFERENCE && relativeangle1 + relativeangle2 <= M_PI){
                                point1.property = Point::concave;
                                point2.property = Point::concave;
                            }else if(abs(wallangle-M_PI/2) < ANGLEMERGEDIFFERENCE){
                                point1.property = Point::convex;
                                point2.property = Point::convex;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Perception::removepointproperties(Map &map, Object::types type){

    //Reset point properties
    for(unsigned long li = 0; li < map.objects.size(); li++){
        Object& obj = map.objects[li];
        if(obj.type == type){
            for(unsigned int pi = 0; pi < obj.points.size(); pi++){
                Point& point = obj.points[pi];
                point.property = Point::floating;
                point.connectedwith.clear();
                point.projectionpoint = NULL;
                point.parents.clear();
            }
        }
    }
}

void Perception::transform(Map& map, vector<node>& nodelist, vector<node>& path, float x, float y, float a, bool direction){
    if(direction){
        map.transform(0,0,a);
        map.transform(x,y,0);
        for(unsigned long n = 0; n < nodelist.size(); n++){
            node& curnode = nodelist[n];
            curnode.transform(0,0,a);
            curnode.transform(x,y,0);
        }
        for(unsigned long p = 0; p < path.size(); p++){
            node& curnode = path[p];
            curnode.transform(0,0,a);
            curnode.transform(x,y,0);
        }
    }else{
        map.transform(x,y,0);
        map.transform(0,0,a);
        for(unsigned long n = 0; n < nodelist.size(); n++){
            node& curnode = nodelist[n];
            curnode.transform(x,y,0);
            curnode.transform(0,0,a);
        }
        for(unsigned long p = 0; p < path.size(); p++){
            node& curnode = path[p];
            curnode.transform(x,y,0);
            curnode.transform(0,0,a);
        }
    }
}
