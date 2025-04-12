
#pragma once

using namespace std;


class Point
{
public:
    double Xcord, Ycord;
    Point(){}
    Point(double X, double Y) //: Point()
    {
        Xcord = X;
        Ycord = Y;
    }
    ~Point() {}
};

class Pose
{
public:
    Point Target;
    double angle;
    Pose() {}
    Pose(Point Pos, double theta)
    {
        Target = Pos;
        angle = theta;
    }
    ~Pose() {}
};

class Line
{
public:
    pair<Point, Point> LinePoints;
    Line() {}
    Line(Point A, Point B)
    {
        LinePoints.first = A;
        LinePoints.second = B;
    }
    ~Line()
    {
    }
};



class Path
{
public:
    double pathlength;
    vector <Point*> PathPoints;
    Path() {}
    ~Path() {}
    void calcPathLength();
};

class Field
{
    private:
    vector<Point*> Path2Snap2;//
    vector<const Point*> Field_Obstacles; //
    vector<const Point*> Red_Pos_Zone; //
    vector<const Point*> Red_Neg_Zone; //
    vector<const Point*> Blue_Pos_Zone; //
    vector<const Point*> Blue_Neg_Zone; //
 
    vector<const Point*> Isolation_Zone;

    
    //vector<const Point*> Offensive_Zone;
    vector<const Point*> MG_Scoring_Points;
    vector<const Point*> MG_Descoring_Points;
    vector<const Point*> WS_Scoring_Points;


    //vector<const Point*> Front_Scoring_Zone;
    vector<const Point*> Intake_Zone;
    Point* Calc_Offest_Point();
    pair<Point*, double> Dist_from_Node(int NodePos, Point* freePoint);
    Line FindOffsetLines(Point*P1, Point* P2, bool offsettype);
    int getIndex(Point* AdjPoint);
    Point* Find_Point_on_Path(Point* freePoint);
   
    
public:
    bool Red_Side;
    bool Blue_Side;
    double Width_Offset;
    double Intake_Offset;
    double MG_Offset;
    pair<Point*,double> Score_Left;
    pair<Point*,double> Score_Right;
    Line* Score_Front;
    Line* Drop_Line; 
    Point* HangPos;
    Point* ML_Point;
 

    Field(double Robot_Width, double Front_Offset, double Rear_Offset, double Arm_Offset);
    Point* Find_Scoring_Pos();
    Point* Find_Drop_Pos();
    
    
    //bool Check_Barrier_Intersects(Point* point, Point* inPath, bool checkoffsets);
    bool CheckCircleIntersection(Point* lineStart, Point* lineEnd, const Point* circleCenter, double radius);
    bool Check_Obstacle_Intersects(Point* point, Point* inPath, bool checkoffsets);
    void Updtae_Intake_Zone();
    bool In_Iso_Zone(float Ball_x, float Ball_y, bool check);
    bool In_Scored_Corner(float MG_x, float MG_y);
    bool In_Descored_Corner(float MG_x, float MG_y);
    bool Near_Intake(float Ball_x, float Ball_y);
    Path Create_Path_to_Target(Point* Current, Point* Target);

      
};