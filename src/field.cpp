#include "robot-config.h"

static double
    Field_XY_Lim = 178.308,
    MatchLoad_Corner = 115.89522, 
    Alley_X = 62.4586,
    Alley_Y = 119.6975,
    CenterXY = 0.00,
    CenterBar_Y = 116.68125;

static Point
    Q1_Field_Corner(Field_XY_Lim, Field_XY_Lim),
    Q2_Field_Corner(Field_XY_Lim, -Field_XY_Lim),
    Q3_Field_Corner(-Field_XY_Lim, -Field_XY_Lim),
    Q4_Field_Corner(-Field_XY_Lim, Field_XY_Lim);
static Line
    Front_Side_Field(Q4_Field_Corner, Q1_Field_Corner),
    Right_Side_Field(Q1_Field_Corner, Q2_Field_Corner),
    Rear_Side_Field(Q2_Field_Corner, Q3_Field_Corner),
    Left_Side_Field(Q3_Field_Corner, Q4_Field_Corner);
static Point
    Match_LoadA(Field_XY_Lim,MatchLoad_Corner),
    Match_LoadB(MatchLoad_Corner,Field_XY_Lim);

static Point
    Center_Bar1(CenterXY, -CenterBar_Y),
    Center_Bar2(CenterXY, CenterBar_Y),
    Blue_Alley1(-Alley_X, Alley_Y),
    Blue_Alley2(Alley_X, Alley_Y),
    Red_Alley1(-Alley_X, -Alley_Y),
    Red_Alley2(Alley_X, -Alley_Y);
static Line
    Center_Bar(Center_Bar1, Center_Bar2),
    Blue_Alley(Blue_Alley1, Blue_Alley2),
    Red_Alley(Red_Alley1, Red_Alley2);
/////////////////////////////////////////////////////////////////////////////
// Goals Ref Cordinates(cm)
static double
    Goal_X = 109.8296,
    Goal_Y = 69.70395;

static Point
    Red_FL_Corner(Goal_X, Goal_Y),        // Front Left Corner
    Red_BL_Corner(Field_XY_Lim, Goal_Y),  // Back Left Corner
    Red_FR_Corner(Goal_X, -Goal_Y),       // Front Right Corner
    Red_BR_Corner(Field_XY_Lim, -Goal_Y); // Back Right Corner
static Line
    Red_Front_Side(Red_FL_Corner, Red_FR_Corner), // Front Red Line
    Red_Left_Side(Red_FL_Corner, Red_BL_Corner),  // Left Red Line
    Red_Right_Side(Red_FR_Corner, Red_BR_Corner); // Right Red Line

static Point
    Blue_FL_Corner(-Goal_X, -Goal_Y),       // Front Left Corner
    Blue_BL_Corner(-Field_XY_Lim, -Goal_Y), // Back Left Corner
    Blue_FR_Corner(-Goal_X, Goal_Y),        // Right - Front Blue corner
    Blue_BR_Corner(-Field_XY_Lim, Goal_Y);  // Right - Rear Blue corner
static Line
    Blue_Front_Side(Blue_FL_Corner, Blue_FR_Corner), // Front Blue Line
    Blue_Left_Side(Blue_FL_Corner, Blue_BL_Corner),  // Left Blue Line
    Blue_Right_Side(Blue_FR_Corner, Blue_BR_Corner); // Right Blue Line
/////////////////////////////////////////////////////////////////////////////
static double
    Scoring_Ref_X = 58.7248,
    Scoring_Ref_Y = 29.3687,
    Nuetral_Y = 20.48, 
    Scoring_Zone_X = 109.8296,
    Side_Score_X = 148.9964,
    Side_Score_Y = 95.40875;
// Scoring Line and Points      // Nuetral_Y was 30.48
static Point
    // Front Scoring Line Points
    Blue_ScoreA(-Scoring_Ref_X,Scoring_Ref_Y),
    Blue_ScoreB(-Scoring_Ref_X,-Scoring_Ref_Y),
    Red_ScoreA(Scoring_Ref_X,Scoring_Ref_Y),
    Red_ScoreB(Scoring_Ref_X,-Scoring_Ref_Y),
    //Side Scoring Points
    L_Red_Score(Side_Score_X,Side_Score_Y),
    R_Red_Score(Side_Score_X,-Side_Score_Y),
    L_Blue_Score(-Side_Score_X,-Side_Score_Y),
    R_Blue_Score(-Side_Score_X,Side_Score_Y);
static Line
    Red_Score(Red_ScoreA,Red_ScoreB),
    Blue_Score(Blue_ScoreA,Blue_ScoreB);
// Isolation Points
static Point
    C_Nuetral_Top(CenterXY,Nuetral_Y),
    C_Nuetral_Bottom(CenterXY,-Nuetral_Y),
    C_Front(0.0, Field_XY_Lim),
    C_Back(0.0, -Field_XY_Lim),
    Nuetral_Left(-Field_XY_Lim,-Nuetral_Y),
    Nuetral_Right(Field_XY_Lim,Nuetral_Y);
// Scoring Zone Points
static Point 
    Front_Red_1(Scoring_Zone_X,CenterBar_Y),
    Front_Red_2(Scoring_Zone_X,-CenterBar_Y),
    Front_Blue_1(-Scoring_Zone_X,CenterBar_Y),
    Front_Blue_2(-Scoring_Zone_X,-CenterBar_Y),
    Back_RB_1(0.00, CenterBar_Y),
    Back_RB_2(0.00, -CenterBar_Y);
//Field Barriers
static Barrier
    RedGoal(Red_Front_Side, Red_Left_Side, Red_Right_Side),
    BlueGoal(Blue_Front_Side, Blue_Left_Side, Blue_Right_Side),
    CenterBarrier(Blue_Alley, Red_Alley, Center_Bar);

// Path 2 Snap 2 Points
static double
    Match_Load_Ref_X = 95.00,
    Match_Load_Ref_Y = 153.010875,
    Match_Load_Center_XY = 124.005438,
    Goal_Zone_XY = 58.334275;
static Point
    Q1_Alley(Match_Load_Ref_X, Match_Load_Ref_Y),
    Q1_Match_Load_Center(Match_Load_Center_XY, Match_Load_Center_XY),
    Q1_Goal_Zone(Goal_Zone_XY, Goal_Zone_XY),
    Q2_Goal_Zone(Goal_Zone_XY, -Goal_Zone_XY),
    Q2_Match_Load_Center(Match_Load_Center_XY, -Match_Load_Center_XY),
    Q2_Alley(Match_Load_Ref_X, -Match_Load_Ref_Y),
    Q3_Alley(-Match_Load_Ref_X, -Match_Load_Ref_Y),
    Q3_Match_Load_Center(-Match_Load_Center_XY, -Match_Load_Center_XY),
    Q3_Goal_Zone(-Goal_Zone_XY, -Goal_Zone_XY),
    Q4_Goal_Zone(-Goal_Zone_XY, Goal_Zone_XY),
    Q4_Match_Load_Center(-Match_Load_Center_XY, Match_Load_Center_XY),
    Q4_Alley(-Match_Load_Ref_X, Match_Load_Ref_Y);

//Hang Points
static double 
    Vert_Hang_X = 103.308,
    Hor_Hang_X = 10.00,
    Hor_Hang_y = 158.308;

static double
    Drop_Off_X = 45.00,
    Drop_Off_Y = 45.00;

static Point
    Red_Drop_A(Drop_Off_X,Drop_Off_Y),
    Red_Drop_B(Drop_Off_X, -Drop_Off_Y),
    Blue_Drop_A(-Drop_Off_X, Drop_Off_Y),
    Blue_Drop_B(-Drop_Off_X,-Drop_Off_Y);

static Line
    Red_Drop(Red_Drop_A,Red_Drop_B),
    Blue_Drop(Blue_Drop_A,Blue_Drop_B);




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double distanceTo(Point *Point1, Point *Point2)
{
    //fprintf(fp,"Point 1 (%.2f,%.2f), Point 2(%.2f,%.2f)", Point1->Xcord, Point1->Ycord, Point2->Xcord, Point2->Ycord);
    double distance = sqrt((Point2->Xcord - Point1->Xcord) *(Point2->Xcord - Point1->Xcord)  + (Point2->Ycord - Point1->Ycord) * (Point2->Ycord - Point1->Ycord));
    return distance;
}

void Path::calcPathLength()
{
    double CalcDist;
    pathlength = 0; 
    for (int i = 0; i < PathPoints.size() - 1; i++)
    {
        CalcDist =  distanceTo(PathPoints[i], PathPoints[i + 1]);
        pathlength += CalcDist;
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Field::Field(double Robot_Width, double Intake_Offset)
{

    if (Side == TeamColor::RED)
    {
        Blue_Side = false; 
        Red_Side = true;

        Isolation_Zone.push_back(&C_Nuetral_Top);
        Isolation_Zone.push_back(&Nuetral_Right);
        Isolation_Zone.push_back(&Q2_Field_Corner);
        Isolation_Zone.push_back(&C_Back);

        Offensive_Zone.push_back(&C_Front);
        Offensive_Zone.push_back(&C_Back);
        Offensive_Zone.push_back(&Q2_Field_Corner);
        Offensive_Zone.push_back(&Q1_Field_Corner);

        Scoring_Zone.push_back(&Front_Red_1);
        Scoring_Zone.push_back(&Front_Red_2);
        Scoring_Zone.push_back(&Back_RB_2);
        Scoring_Zone.push_back(&Back_RB_1);

        Score_Front = &Red_Score;
        Score_Left.first = &L_Red_Score;
        Score_Left.second = 180.0;
        Score_Right.first = &R_Red_Score;
        Score_Right.second = 0.0;

        Drop_Line = &Blue_Drop;
        ML_Point = &Q3_Match_Load_Center;

        #if defined(MANAGER_ROBOT)
        HangPos = new Point(-Vert_Hang_X, -Alley_Y);
        #else
        HangPos = new Point(Hor_Hang_X, -Hor_Hang_y);
        #endif
    }
    else 
    {
        Red_Side = false;
        Blue_Side = true;

        Isolation_Zone.push_back(&C_Nuetral_Bottom);
        Isolation_Zone.push_back(&Nuetral_Left);
        Isolation_Zone.push_back(&Q4_Field_Corner);
        Isolation_Zone.push_back(&C_Front);

        Offensive_Zone.push_back(&C_Front);
        Offensive_Zone.push_back(&C_Back);
        Offensive_Zone.push_back(&Q3_Field_Corner);
        Offensive_Zone.push_back(&Q4_Field_Corner);

        Scoring_Zone.push_back(&Front_Blue_1);
        Scoring_Zone.push_back(&Front_Blue_2);
        Scoring_Zone.push_back(&Back_RB_2);
        Scoring_Zone.push_back(&Back_RB_1);
        
        Score_Front = &Blue_Score;
        Score_Left.first = &L_Blue_Score;
        Score_Left.second = 0.00;
        Score_Right.first = &R_Blue_Score;
        Score_Right.second = 180.0;

        Drop_Line = &Red_Drop;
        ML_Point = &Q1_Match_Load_Center;

        #if defined(MANAGER_ROBOT)
        HangPos = new Point(Vert_Hang_X, Alley_Y);
        #else
        HangPos = new Point(-Hor_Hang_X, Hor_Hang_y);
        #endif
    }

    Width_Offset = Robot_Width;
    Front_Offset = Intake_Offset;

    Path2Snap2.push_back(&Q1_Alley);
    Path2Snap2.push_back(&Q1_Match_Load_Center);
    Path2Snap2.push_back(&Q1_Goal_Zone);
    Path2Snap2.push_back(&Q2_Goal_Zone);
    Path2Snap2.push_back(&Q2_Match_Load_Center);
    Path2Snap2.push_back(&Q2_Alley);
    Path2Snap2.push_back(&Q3_Alley);
    Path2Snap2.push_back(&Q3_Match_Load_Center);
    Path2Snap2.push_back(&Q3_Goal_Zone);
    Path2Snap2.push_back(&Q4_Goal_Zone);
    Path2Snap2.push_back(&Q4_Match_Load_Center);
    Path2Snap2.push_back(&Q4_Alley);

    Field_Barriers.push_back(&CenterBarrier);
    Field_Barriers.push_back(&BlueGoal);
    Field_Barriers.push_back(&RedGoal);

    Goal_Zone.push_back(&Red_BL_Corner);
    Goal_Zone.push_back(&Red_FL_Corner);
    Goal_Zone.push_back(&Red_FR_Corner);
    Goal_Zone.push_back(&Red_BR_Corner);

    ML_Zone.push_back(&Q1_Field_Corner);
    ML_Zone.push_back(&Match_LoadA);
    ML_Zone.push_back(&Match_LoadB);



}

Point* Field::Find_Drop_Pos()
{
    Point* PoL;
    double
        Ax = Drop_Line->LinePoints.first.Xcord,
        Ay = Drop_Line->LinePoints.first.Ycord,
        Bx = Drop_Line->LinePoints.second.Xcord,
        By = Drop_Line->LinePoints.second.Ycord,
        Pointx = GPS.xPosition(vex::distanceUnits::cm),
        Pointy = GPS.yPosition(vex::distanceUnits::cm);
    // fprintf(fp,"Line Seg Point A (%.2f, %.2f), Point B (%.2f, %.2f), Point C(%.2f, %.2f)",Ax,Ay,Bx,By,Pointx,Pointy);
    double Px = Bx - Ax;
    double Py = By - Ay;
    double temp = (Px * Px) + (Py * Py);
    double U = ((Pointx - Ax) * Px + (Pointy - Ay) * Py) / (temp);
    if (U > 1)
    {
        U = 1;
    }
    else if (U < 0)
    {
        U = 0;
    }
    double X = Ax + U * Px;
    double Y = Ay + U * Py;
    // double Dx = X - Pointx;
    // double Dy = Y - Pointy;
    //double Dist = sqrt((Dx * Dx) + (Dy * Dy));
   
    PoL = new Point(X, Y);
    //fprintf(fp,"\rPoint on Line:(%.2f, %.2f) Distance:\n",X, Y);

    return PoL;
}

Point* Field::Find_Scoring_Pos()
{
    Point* PoL;
    double
        Ax = Score_Front->LinePoints.first.Xcord,
        Ay = Score_Front->LinePoints.first.Ycord,
        Bx = Score_Front->LinePoints.second.Xcord,
        By = Score_Front->LinePoints.second.Ycord,
        Pointx = GPS.xPosition(vex::distanceUnits::cm),
        Pointy = GPS.yPosition(vex::distanceUnits::cm);
    // fprintf(fp,"Line Seg Point A (%.2f, %.2f), Point B (%.2f, %.2f), Point C(%.2f, %.2f)",Ax,Ay,Bx,By,Pointx,Pointy);
    double Px = Bx - Ax;
    double Py = By - Ay;
    double temp = (Px * Px) + (Py * Py);
    double U = ((Pointx - Ax) * Px + (Pointy - Ay) * Py) / (temp);
    if (U > 1)
    {
        U = 1;
    }
    else if (U < 0)
    {
        U = 0;
    }
    double X = Ax + U * Px;
    double Y = Ay + U * Py;
    // double Dx = X - Pointx;
    // double Dy = Y - Pointy;
    //double Dist = sqrt((Dx * Dx) + (Dy * Dy));
   
    PoL = new Point(X, Y);
    //fprintf(fp,"\rPoint on Line:(%.2f, %.2f) Distance:\n",X, Y);

    return PoL;
}
Point* Field::Calc_Offest_Point()
{
    double 
    X_pos = GPS.xPosition(vex::distanceUnits::cm),
    Y_pos = GPS.yPosition(vex::distanceUnits::cm),
    theta = GPS.heading(vex::rotationUnits::deg);

    double dX = cos(theta) * Front_Offset;
    double dY = sin(theta) * Front_Offset;

    return new Point(X_pos+dX,Y_pos+dY);

}

int orientation(Point* p, Point* q, Point* r)
{
    double val = (q->Ycord - p->Ycord) * (r->Xcord - q->Xcord) - (q->Xcord - p->Xcord) * (r->Ycord - q->Ycord);
    if (val == 0)
        return 0; // collinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool Check_Intersects(Point* CurrentPos, Point* PointinPath, Line BarrierLine)
{
    Point Line_PointA = BarrierLine.LinePoints.first;
    Point Line_PointB = BarrierLine.LinePoints.second;
    int o1 = orientation(CurrentPos, PointinPath, &Line_PointA);
    int o2 = orientation(CurrentPos, PointinPath, &Line_PointB);
    int o3 = orientation(&Line_PointA, &Line_PointB, CurrentPos);
    int o4 = orientation(&Line_PointA, &Line_PointB, PointinPath);
    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    return false; // Doesn't fall in any of the above cases
}

Line Field::FindOffsetLines(Point* P1, Point* P2, bool offsettype)
{
    double dx = P2->Xcord - P1->Xcord;
    double dy = P2->Ycord - P1->Ycord;
    double Length = sqrt(pow(dx,2) + pow(dy,2));
    double Dx = dy * Width_Offset/Length;
    double Dy = -dx * Width_Offset/Length;
    Point ModP1;
    Point ModP2;

    if(offsettype)
    {
        ModP1 = Point(P1->Xcord+Dx,P1->Ycord+Dy);
        ModP2 = Point(P2->Xcord+Dx,P2->Ycord+Dy);
    }
    else
    {
        ModP1 = Point(P1->Xcord-Dx,P1->Ycord-Dy);
        ModP2 = Point(P2->Xcord-Dx,P2->Ycord-Dy);
    }

    Line ParallelLine(ModP1,ModP2);

    return ParallelLine;
}

bool Field::Check_Barrier_Intersects(Point* point, Point* inPath, bool checkoffsets = false)
{   
    Line LineA;
    Line LineB;
    bool Intersect = false;

    if(checkoffsets)
    {
    LineA = FindOffsetLines(point,inPath,true);
    LineB = FindOffsetLines(point,inPath,false);
    }

    for (int i = 0; i < Field_Barriers.size(); i++)
    {
        for (int j = 0; j < Field_Barriers[i]->BarrierLines.size(); j++)
        {
            if (Check_Intersects(point, inPath, Field_Barriers[i]->BarrierLines[j]))
            {
                return true;
            }
            if(checkoffsets)
            {
                if(Check_Intersects(&LineA.LinePoints.first, &LineA.LinePoints.second, Field_Barriers[i]->BarrierLines[j]))
                {
                    return true;
                }
                if(Check_Intersects(&LineB.LinePoints.first, &LineB.LinePoints.second, Field_Barriers[i]->BarrierLines[j]))
                {
                    return true;
                }
            }
        }
    }
    return Intersect;
}

void Field::Updtae_Intake_Zone()
{
    Point Current(GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm));
    Point* Intake_Off = Calc_Offest_Point();
    //fprintf(fp,"")
    Line Intake_OffsetA = FindOffsetLines(&Current, Intake_Off,true);
    Line Intake_OffsetB = FindOffsetLines(&Current, Intake_Off,true);
    Point A1 = Intake_OffsetA.LinePoints.first;
    Point A2 = Intake_OffsetA.LinePoints.second;
    Point B1 = Intake_OffsetA.LinePoints.first;
    Point B2 = Intake_OffsetA.LinePoints.second;
    Intake_Zone.clear();
    Intake_Zone.push_back(&A1);
    Intake_Zone.push_back(&A2);
    Intake_Zone.push_back(&B2);
    Intake_Zone.push_back(&B1);
    //fprintf(fp,"\rIntake Zone Points, A1:(%.2f, %.2f) A2:(%.2f, %.2f) || B1:(%.2f, %.2f) B2:(%.2f, %.2f)\n", A1.Xcord, A1.Ycord, A2.Xcord, A2.Ycord, B1.Xcord, B1.Ycord, B2.Xcord, B2.Ycord);

}

bool Inside_Polygon(float tar_x, float tar_y, vector <const Point*> Zone)
{
    tar_x = tar_x * 100;
    tar_y = tar_y * 100;
    int num_vertices = Zone.size();
    float x = tar_x, y = tar_y;
    bool inside = false;
    Point P1(Zone[0]->Xcord, Zone[0]->Ycord);
    Point P2;
    // Loop through each edge in the polygon
    for (int i = 1; i <= num_vertices; i++)
    {
        P2 = Point(Zone[i % num_vertices]->Xcord, Zone[i % num_vertices]->Ycord);
        if (y > min(P1.Ycord, P2.Ycord))
        {
            if (y <= max(P1.Ycord, P2.Ycord))
            {
                if (x <= max(P1.Xcord, P2.Xcord))
                {
                    double x_intersection = (y - P1.Ycord) * (P2.Xcord - P1.Xcord) / (P2.Ycord - P1.Ycord) + P1.Xcord;
                    if (P1.Xcord == P2.Xcord || x <= x_intersection)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        P1 = P2;
    }
    return inside;
}

bool Field::In_Goal_Zone(float Ball_x, float Ball_y)
{
    Ball_x = fabs(Ball_x);
    return Inside_Polygon(Ball_x, Ball_y, Goal_Zone);
}

bool Field::In_MatchLoad_Zone(float Ball_x, float Ball_y)
{
    Ball_x = fabs(Ball_x);
    Ball_y = fabs(Ball_y);
    return Inside_Polygon(Ball_x, Ball_y, ML_Zone);
}

bool Field::In_Offensive_Zone(float Ball_x, float Ball_y, bool check)
{   if(check)
        return Inside_Polygon(Ball_x, Ball_y, Offensive_Zone);
    else 
        check = true;
    return check;
}
bool Field::In_Front_Score_Zone()
{   
    return Inside_Polygon((GPS.xPosition(vex::distanceUnits::cm )/100), (GPS.yPosition(vex::distanceUnits::cm)/100),Scoring_Zone);
}


bool Field::In_Iso_Zone(float Ball_x, float Ball_y, bool check)
{
    if(check)
        return Inside_Polygon(Ball_x, Ball_y, Isolation_Zone);
    else
        check = true;
    return check;
}

bool Field::Near_Intake(float Ball_x, float Ball_y)
{
    return Inside_Polygon(Ball_x, Ball_y,Intake_Zone);
}

bool pairCompare(const std::pair<Point*, double> &firstElem, const std::pair<Point*, double> &secondElem)
{
    return firstElem.second < secondElem.second;
}
pair<Point*, double> Field::Dist_from_Node(int NodePos, Point* freepoint)
{
    pair<Point*,double> temp(Path2Snap2[NodePos],distanceTo(freepoint,Path2Snap2[NodePos]));
    return temp; 
}
Point* Field::Find_Point_on_Path(Point* freePoint)
{
    vector<pair<Point*, double>> Point_Dist; // Vector that stores a Point which lies on a line in our Path2Snap2 and the distance from target
    Point* in_Path = freePoint;             // This holds the closest point to a line on the Snap2path and its line position on that path

    for (int i = 0; i < Path2Snap2.size(); i++)
    {
        Point_Dist.push_back(Dist_from_Node(i,freePoint));
    }
    sort(Point_Dist.begin(), Point_Dist.end(), pairCompare);
    // for(int i = 0; i < Point_Dist.size(); i++)
    // {
    //     fprintf(fp,"\rPoint in Path:(%.2f, %.2f) distance from point %.2f cm\n", Point_Dist[i].first->Xcord, Point_Dist[i].first->Ycord, Point_Dist[i].second);
    // }
    for (int j = 0; j < Point_Dist.size(); j++)
    {
        //fprintf(fp,"\nPoint in Path:(%.2f, %.2f) distance from point (%.2f, %.2f): %.2f cm\n", Point_Dist[j].first->Xcord, Point_Dist[j].first->Xcord, freePoint->Xcord, freePoint->Ycord, Point_Dist[j].second);
        if (!Check_Barrier_Intersects(freePoint, Point_Dist[j].first, true))
        {
            in_Path = Point_Dist[j].first;
            break;
        }
    }
    return in_Path;
}

int Field::getIndex(Point* AdjPoint) 
{ 
    int index = -1;
    auto it = find(Path2Snap2.begin(), Path2Snap2.end(), AdjPoint); 
  
    // If element was found 
    if (it != Path2Snap2.end())  
    { 
        index = it - Path2Snap2.begin(); 
    } 
    return index + 1;
} 

Path Field::Create_Path_to_Target(Point* Current, Point* Target)
{
    Path DrivePath;
    Point* Start = Find_Point_on_Path(Current);
    Point* End = Find_Point_on_Path(Target);
    fprintf(fp, "\rCurrent Pos: (%.2f, %.2f) - > Target Pos: (%.2f, %.2f)\n", Current->Xcord, Current->Ycord, Target->Xcord, Target->Ycord);
    // fprintf(fp, "\rFirst point to drive to is (%.2f, %.2f)\n", Start->Xcord, Start->Ycord);
    // fprintf(fp, "\rLast point before target (%.2f, %.2f)\n", End->Xcord, End->Ycord);
    int 
    StartIndex = getIndex(Start),
    EndIndex = getIndex(End);

    Path PathA; // Clockwise 
    PathA.PathPoints.push_back(Start);
    for(int i = StartIndex; i != EndIndex ; i++)
    {
        if (i == Path2Snap2.size())
        {
            i = 0;
        }
        PathA.PathPoints.push_back(Path2Snap2[i]);
    }
    //PathA.PathPoints.push_back(End);
    PathA.PathPoints.push_back(Target);
    PathA.calcPathLength();

    Path PathB; // Clockwise 
    //PathB.PathPoints.push_back(Start);
    for(int i = StartIndex - 1; i != EndIndex - 1  ; i--)
    {
        PathB.PathPoints.push_back(Path2Snap2[i]);
        if (i == 0)
        {
            i = Path2Snap2.size();
        }
    }
    PathB.PathPoints.push_back(End);
    PathB.PathPoints.push_back(Target);
    PathB.calcPathLength();

    if(PathA.pathlength < PathB.pathlength)
    {
        for(int i = 0; i < PathA.PathPoints.size(); i++)
        {
            DrivePath.PathPoints.push_back(PathA.PathPoints[i]);
        }
    }
    else
    {
        for(int i = 0; i < PathB.PathPoints.size(); i++)
        {
            DrivePath.PathPoints.push_back(PathB.PathPoints[i]);
        }
    }

    for(int i = 0; i < DrivePath.PathPoints.size(); i++)
    {
        fprintf(fp,"\rPoint in Path:(%.2f, %.2f) ->\n", DrivePath.PathPoints[i]->Xcord, DrivePath.PathPoints[i]->Ycord);
    }
    return DrivePath;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Old in this file but it didnt work on the 15 inch
// Path Field::Create_Path_to_Target(Point* Current, Point* Target)
// {
//     Path DrivePath;
//     Point* Start = Find_Point_on_Path(Current);
//     Point* End = Find_Point_on_Path(Target);
//     fprintf(fp, "\rCurrent Pos: (%.2f, %.2f) - > Target Pos: (%.2f, %.2f)\n", Current->Xcord, Current->Ycord, Target->Xcord, Target->Ycord);
//     fprintf(fp, "\rFirst point to drive to is (%.2f, %.2f)\n", Start->Xcord, Start->Ycord);
//     fprintf(fp, "\rLast point before target (%.2f, %.2f)\n", End->Xcord, End->Ycord);
//     int 
//     StartIndex = getIndex(Start),
//     EndIndex = getIndex(End);


//     Path PathA; // Clockwise 
//     PathA.PathPoints.push_back(Start);
//     for(int i = StartIndex + 1; i != EndIndex ; i++)
//     {
//         PathA.PathPoints.push_back(Path2Snap2[i-1]);
//         if (i == Path2Snap2.size())
//         {
//             i = 0;
//         }
        
//     }
//     if(Check_Barrier_Intersects(PathA.PathPoints.back(), Target, true)){
//         PathA.PathPoints.push_back(End);
//     }
//     PathA.calcPathLength();


//     Path PathB; // Counter-Clockwise 
//     PathB.PathPoints.push_back(Start);
//     for(int i = StartIndex - 1; i != EndIndex  ; i--)
//     {
//         if (i == 0)
//         {
//             i = Path2Snap2.size();
//         }
//         PathB.PathPoints.push_back(Path2Snap2[i-1]);
//     }
//     if(Check_Barrier_Intersects(PathB.PathPoints.back(), Target, true)){
//         PathB.PathPoints.push_back(End);
//     }
//     PathB.calcPathLength();


//     if(PathA.pathlength < PathB.pathlength)
//     {
//         for(int i = 0; i < PathA.PathPoints.size(); i++)
//         {
//             DrivePath.PathPoints.push_back(PathA.PathPoints[i]);
//         }
//     }
//     else
//     {
//         for(int i = 0; i < PathB.PathPoints.size(); i++)
//         {
//             DrivePath.PathPoints.push_back(PathB.PathPoints[i]);
//         }
//     }
//     DrivePath.PathPoints.push_back(Target);
//     fprintf(fp,"\rFirst Point om Drive Path\n");
//     for(int i = 0; i < DrivePath.PathPoints.size() - 1; i++)
//     {
//         int tempIndex = getIndex(DrivePath.PathPoints[i]);
//         fprintf(fp,"\rPath Index: %i ->\n", tempIndex);
//     }
//     fprintf(fp,"\rTarget Point om Drive Path\n");
//     return DrivePath;
// }



/////////////////////////////////////////////////////////////////////////////////////////////////////

//worked on the 24 inch robot in the old file
// Path Field::Create_Path_to_Target(Point* Current, Point* Target)
// {
//     Path DrivePath;
//     Point* Start = Find_Point_on_Path(Current);
//     Point* End = Find_Point_on_Path(Target);
//     fprintf(fp, "\rCurrent Pos: (%.2f, %.2f) - > Target Pos: (%.2f, %.2f)\n", Current->Xcord, Current->Ycord, Target->Xcord, Target->Ycord);
//     fprintf(fp, "\rFirst point to drive to is (%.2f, %.2f)\n", Start->Xcord, Start->Ycord);
//     fprintf(fp, "\rLast point before target (%.2f, %.2f)\n", End->Xcord, End->Ycord);
//     int 
//     StartIndex = getIndex(Start),
//     EndIndex = getIndex(End);


//     Path PathA; // Clockwise 
//     PathA.PathPoints.push_back(Start);
//     for(int i = StartIndex + 1; i != EndIndex ; i++)
//     {
//         PathA.PathPoints.push_back(Path2Snap2[i-1]);
//         if (i == Path2Snap2.size())
//         {
//             i = 0;
//         }
        
//     }
//     if(Check_Barrier_Intersects(PathA.PathPoints.back(), Target, true)){
//         PathA.PathPoints.push_back(End);
//     }
//     PathA.calcPathLength();


//     Path PathB; // Counter-Clockwise 
//     PathB.PathPoints.push_back(Start);
//     for(int i = StartIndex - 1; i != EndIndex  ; i--)
//     {
//         if (i == 0)
//         {
//             i = Path2Snap2.size();
//         }
//         PathB.PathPoints.push_back(Path2Snap2[i-1]);
//     }
//     if(Check_Barrier_Intersects(PathB.PathPoints.back(), Target, true)){
//         PathB.PathPoints.push_back(End);
//     }
//     PathB.calcPathLength();


//     if(PathA.pathlength < PathB.pathlength)
//     {
//         for(int i = 0; i < PathA.PathPoints.size(); i++)
//         {
//             DrivePath.PathPoints.push_back(PathA.PathPoints[i]);
//         }
//     }
//     else
//     {
//         for(int i = 0; i < PathB.PathPoints.size(); i++)
//         {
//             DrivePath.PathPoints.push_back(PathB.PathPoints[i]);
//         }
//     }
//     DrivePath.PathPoints.push_back(Target);
//     fprintf(fp,"\rFirst Point om Drive Path\n");
//     for(int i = 0; i < DrivePath.PathPoints.size() - 1; i++)
//     {
//         int tempIndex = getIndex(DrivePath.PathPoints[i]);
//         fprintf(fp,"\rPath Index: %i ->\n", tempIndex);
//     }
//     fprintf(fp,"\rTarget Point om Drive Path\n");
//     return DrivePath;
// }