#include "robot-config.h"

// Class Variables(cm)
static double
    Field_XY_Lim = 178.308,
    CenterXY = 0.00,
    //Scoring_Cord = 110, 
    Path_Cord_1 = 25.4,
    Path_Cord_2 = 118.284,
    Ladder_Cord = 60.96,
    Scoring_Ref_1 = 142.88,
    Scoring_Ref_2 = 182.88;



// Field 
static Point // Q for Quandrants
    Q1_Field_Corner(Field_XY_Lim, Field_XY_Lim),
    Q2_Field_Corner(Field_XY_Lim, -Field_XY_Lim),
    Q3_Field_Corner(-Field_XY_Lim, -Field_XY_Lim),
    Q4_Field_Corner(-Field_XY_Lim, Field_XY_Lim);
static Line
    Front_Side_Field(Q4_Field_Corner, Q1_Field_Corner),
    Right_Side_Field(Q1_Field_Corner, Q2_Field_Corner),
    Rear_Side_Field(Q2_Field_Corner, Q3_Field_Corner),
    Left_Side_Field(Q3_Field_Corner, Q4_Field_Corner);

// Ladder
static Point 
    Front_Ladder_Post(CenterXY, Ladder_Cord),
    Right_Ladder_Post(Ladder_Cord, CenterXY),
    Rear_Ladder_Post(CenterXY, -Ladder_Cord),
    Left_Ladder_Post(-Ladder_Cord, CenterXY);

//Path Points 
static Point
    PP1(Path_Cord_1, Path_Cord_2),
    PP2(Path_Cord_2, Path_Cord_1),
    PP3(Path_Cord_2, -Path_Cord_1),
    PP4(Path_Cord_1, -Path_Cord_2),
    PP5(-Path_Cord_1, -Path_Cord_2),
    PP6(-Path_Cord_2, -Path_Cord_1),
    PP7(-Path_Cord_2, Path_Cord_1),
    PP8(-Path_Cord_1, Path_Cord_2);


//Isolation Ref Points
static Point
    Center_Front(CenterXY,Field_XY_Lim),
    Center_Rear(CenterXY,-Field_XY_Lim);

// Scoring Zone Points
static Point
    Q1_Scoring_1(Scoring_Ref_2, Scoring_Ref_1),
    Q1_Scoring_2(Scoring_Ref_1, Scoring_Ref_2),
    Q2_Scoring_1(-Scoring_Ref_1, Scoring_Ref_2),
    Q2_Scoring_2(-Scoring_Ref_2, Scoring_Ref_1),
    Q3_Scoring_1(-Scoring_Ref_2, -Scoring_Ref_1),
    Q3_Scoring_2(-Scoring_Ref_1, -Scoring_Ref_2),
    Q4_Scoring_1(Scoring_Ref_1, -Scoring_Ref_2),
    Q4_Scoring_2(Scoring_Ref_2, -Scoring_Ref_1);






    
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

Field::Field(double Robot_Width, double Front_Offset, double Rear_Offset, double Arm_Offset)
{
    static Point Alliance_WS;

    if (Side == TeamColor::RED)
    {
        Blue_Side = false; 
        Red_Side = true;

        Isolation_Zone = {&Center_Front, &Center_Rear, &Q3_Field_Corner, &Q2_Field_Corner};

        Alliance_WS = Point((Arm_Offset - Field_XY_Lim), CenterXY);
    }
    else 
    {
        Red_Side = false;
        Blue_Side = true;

        Isolation_Zone = {&Center_Front, &Center_Rear, &Q4_Field_Corner, &Q1_Field_Corner};

        Alliance_WS = Point((Field_XY_Lim - Arm_Offset), CenterXY);
    
    }

    Width_Offset = Robot_Width;
    Intake_Offset = Front_Offset;
    MG_Offset = Rear_Offset;

    Path2Snap2 = {&PP1, &PP2, &PP3, &PP4, &PP5, &PP6, &PP7, &PP8};
   
    Field_Obstacles = {&Front_Ladder_Post, &Right_Ladder_Post,
                       &Rear_Ladder_Post, &Left_Ladder_Post};

    Red_Pos_Zone = {&Q3_Scoring_1, &Q3_Scoring_2, &Q3_Field_Corner};
    Red_Neg_Zone = {&Q2_Scoring_1, &Q2_Scoring_2, &Q2_Field_Corner};

    Blue_Pos_Zone = {&Q4_Scoring_1, &Q4_Scoring_2, &Q4_Field_Corner};
    Blue_Neg_Zone = {&Q1_Scoring_1, &Q1_Scoring_2, &Q1_Field_Corner};

    static Point Front_WS(CenterXY, (Field_XY_Lim - Arm_Offset));
    static Point Rear_WS(CenterXY, (Arm_Offset - Field_XY_Lim));

    WS_Scoring_Points = {&Alliance_WS, &Front_WS, &Rear_WS};

}

Point* Field::Find_Drop_Pos()
{
    Point* PoL;
    double
        Ax = Drop_Line->LinePoints.first.Xcord,
        Ay = Drop_Line->LinePoints.first.Ycord,
        Bx = Drop_Line->LinePoints.second.Xcord,
        By = Drop_Line->LinePoints.second.Ycord,
        Pointx = GPS.xPosition(),
        Pointy = GPS.yPosition();
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
        Pointx = GPS.xPosition(),
        Pointy = GPS.yPosition();
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
    X_pos = GPS.xPosition(),
    Y_pos = GPS.yPosition(),
    theta = GPS.heading();

    double dX = cos(theta) * Intake_Offset;
    double dY = sin(theta) * Intake_Offset;

    return new Point(X_pos+dX,Y_pos+dY);

}

int orientation(Point* p, Point* q, Point* r)
{
    double val = (q->Ycord - p->Ycord) * (r->Xcord - q->Xcord) - (q->Xcord - p->Xcord) * (r->Ycord - q->Ycord);
    if (val == 0)
        return 0; // collinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
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

bool Field::CheckCircleIntersection(Point* lineStart, Point* lineEnd, const Point* circleCenter, double radius) 
{
    // Calculate the direction vector of the line
    double dx = lineEnd->Xcord - lineStart->Xcord;
    double dy = lineEnd->Ycord - lineStart->Ycord;
    
    // Calculate coefficients for the quadratic equation
    double a = dx * dx + dy * dy;
    double b = 2 * (dx * (lineStart->Xcord - circleCenter->Xcord) + 
                     dy * (lineStart->Ycord - circleCenter->Ycord));
    double c = (lineStart->Xcord - circleCenter->Xcord) * (lineStart->Xcord - circleCenter->Xcord) +
               (lineStart->Ycord - circleCenter->Ycord) * (lineStart->Ycord - circleCenter->Ycord) -
               radius * radius;
    
    // Calculate discriminant
    double discriminant = b * b - 4 * a * c;
    
    // If discriminant is negative, there's no intersection
    if (discriminant < 0) {
        return false;
    }
    
    // Calculate the parameters where the intersections occur
    double t1 = (-b + sqrt(discriminant)) / (2 * a);
    double t2 = (-b - sqrt(discriminant)) / (2 * a);
    
    // Check if at least one intersection point is on the line segment
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
}


bool Field::Check_Obstacle_Intersects(Point* point, Point* inPath, bool checkoffsets) 
{   
    Line LineA;
    Line LineB;
    bool Intersect = false;

    if(checkoffsets) 
    {
        LineA = FindOffsetLines(point, inPath, true);
        LineB = FindOffsetLines(point, inPath, false);
    }

    // Check for intersection with field obstacles (assuming they're now circles)
    for (int i = 0; i < Field_Obstacles.size(); i++) {
        double obstacleRadius = 10.0; // Replace with your actual obstacle radius

        // Check the main line
        if (CheckCircleIntersection(point, inPath, Field_Obstacles[i], obstacleRadius)) 
        {
            return true;
        }
        
        // Check offset lines if needed
        if(checkoffsets) 
        {
            if(CheckCircleIntersection(&LineA.LinePoints.first, &LineA.LinePoints.second, 
                                      Field_Obstacles[i], obstacleRadius)) 
            {
                return true;
            }
            if(CheckCircleIntersection(&LineB.LinePoints.first, &LineB.LinePoints.second, 
                                      Field_Obstacles[i], obstacleRadius)) 
            {
                return true;
            }
        }
    }
    
    return Intersect;
}

void Field::Updtae_Intake_Zone()
{
    Point Current(GPS.xPosition(), GPS.yPosition());
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



bool Field::In_Scored_Corner(float MG_x, float MG_y)
{
    MG_x = fabs(MG_x);
    MG_y = fabs(MG_y);
    if(Inside_Polygon(MG_x, MG_y, Red_Pos_Zone) || Inside_Polygon(MG_x, MG_y, Blue_Pos_Zone))
        return true;
    else 
        return false;

}

bool Field::In_Descored_Corner(float MG_x, float MG_y)
{
    MG_x = fabs(MG_x);
    MG_y = fabs(MG_y);
    if(Inside_Polygon(MG_x, MG_y, Red_Neg_Zone) || Inside_Polygon(MG_x, MG_y, Blue_Neg_Zone))
        return true;
    else 
        return false;

}



bool Field::In_Iso_Zone(float Obj_x, float Obj_y, bool check)
{
    if(check)
        return Inside_Polygon(Obj_x, Obj_y, Isolation_Zone);
    else
        check = true;
    return check;
}

bool Field::Near_Intake(float Ring_x, float Ring_y)
{
    return Inside_Polygon(Ring_x, Ring_y,Intake_Zone);
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
        if (!Check_Obstacle_Intersects(freePoint, Point_Dist[j].first, true))
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
    fprintf(fp,"\r it is %d\n", it);
  
    // If element was found 
    if (it != Path2Snap2.end()) {
        int index = std::distance(Path2Snap2.begin(), it);
        fprintf(fp, "\r it is at index: %d\n", index);
    } else {
        fprintf(fp, "\r AdjPoint not found in Path2Snap2\n");
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
    int stopper = 0;
    fprintf(fp,"\r StartI: %d\n", StartIndex);
    fprintf(fp,"\r EndI: %d\n", EndIndex);

    Path PathA; // Clockwise 
    PathA.PathPoints.push_back(Start);


    // for(int i = StartIndex; i != EndIndex ; i++)
    // {


    //     if (i == Path2Snap2.size())
    //     {
    //         i = 0;
    //     }

    //     fprintf(fp,"\r I path a: %d\n", i);
    //     PathA.PathPoints.push_back(Path2Snap2[i]);
    // }

    int i = StartIndex;
    while (i != EndIndex) 
    {
        fprintf(fp, "\rI path a: %d\n", i);
        PathA.PathPoints.push_back(Path2Snap2[i]);
    
        i = (i + 1) % Path2Snap2.size(); 
    }


    //PathA.PathPoints.push_back(End);
    PathA.PathPoints.push_back(Target);
    PathA.calcPathLength();

    Path PathB; // Clockwise 
    //PathB.PathPoints.push_back(Start);
    
    while (i != EndIndex) 
    {
        fprintf(fp, "\rI path a: %d\n", i);
        PathA.PathPoints.push_back(Path2Snap2[i]);

        i = (i - 1 + Path2Snap2.size()) % Path2Snap2.size();
    }

    // for(int i = StartIndex - 1; i != EndIndex - 1  ; i--)
    // {
    //     fprintf(fp,"\r I path b : %d\n", i);
    //     PathB.PathPoints.push_back(Path2Snap2[i]);
    //     if (i == 0)
    //     {
    //         i = Path2Snap2.size();
            
            
    //     }
    // }


    PathB.PathPoints.push_back(End);
    PathB.PathPoints.push_back(Target);
    PathB.calcPathLength();

    fprintf(fp,"\r LPath a: %d\n", PathA.pathlength );
    fprintf(fp,"\r LPath b: %d\n", PathB.pathlength );

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