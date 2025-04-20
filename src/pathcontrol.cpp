#include "pathcontrol.h"
#include "robot-config.h"
#include <sstream>


// Parámetros de control
double lookahead_dist = 5; 
double base_speed = 20;      
double turn_k = 0.1; 

// Estructura para representar puntos intermedios con orientación opcional
struct PathPoint {
  Point position;
  double heading_deg; // -1 si no se especifica
  
  PathPoint(double x, double y, double h = -1) : position({x, y}), heading_deg(h) {}
  PathPoint(Point p, double h = -1) : position(p), heading_deg(h) {}
};

std::vector<Point> generateBezierPath(
    Point start,
    double start_heading_deg,
    Point end, 
    double end_heading_deg,
    double control_dist = 100, int steps = 50) 
{
  std::vector<Point> path;

  double sh = start_heading_deg * M_PI / 180.0;
  double eh = end_heading_deg * M_PI / 180.0;

  Point P0 = start;
  Point P3 = end;

  Point P1 = {
    P0.Xcord + cos(sh) * control_dist,
    P0.Ycord + sin(sh) * control_dist
  };

  Point P2 = {
    P3.Xcord - cos(eh) * control_dist,
    P3.Ycord - sin(eh) * control_dist
  };

  // Generar puntos de la curva
  for (int i = 0; i <= steps; i++) {
    double t = (double)i / steps;

    double x = pow(1 - t, 3) * P0.Xcord +
               3 * pow(1 - t, 2) * t * P1.Xcord +
               3 * (1 - t) * pow(t, 2) * P2.Xcord +
               pow(t, 3) * P3.Xcord;

    double y = pow(1 - t, 3) * P0.Ycord +
               3 * pow(1 - t, 2) * t * P1.Ycord +
               3 * (1 - t) * pow(t, 2) * P2.Ycord +
               pow(t, 3) * P3.Ycord;

    path.push_back({x, y});
  }

  return path;
}

// Vector global para almacenar la trayectoria actual
std::vector<Point> path;
  
double distanceTo(Point a, Point b) 
{
    return sqrt(pow(b.Xcord - a.Xcord, 2) + pow(b.Ycord - a.Ycord, 2));
}

double degToRad(double deg) 
{
    return deg * M_PI / 180.0;
}        

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

Point getLookahead(Point current) {
    for (int i = 0; i < path.size(); i++) {
      if (distanceTo(current, path[i]) > lookahead_dist) {
        return path[i];
      }
    }
    return path.back();
}

// Obtiene la posición actual del robot
Point getCurrentPosition() {
    return {GPS.xPosition(), GPS.yPosition()};
}

// Genera una trayectoria con puntos intermedios
std::vector<Point> generatePathWithWaypoints(std::vector<PathPoint> waypoints) {
    std::vector<Point> fullPath;
    
    // Asegurarse de que haya al menos un punto de destino
    if (waypoints.size() < 1) {
        return fullPath;
    }
    
    // Obtener la posición actual como punto de inicio
    Point currentPos = getCurrentPosition();
    double currentHeading = GPS.heading();
    
    // Agregar segmentos Bezier entre cada par de waypoints
    PathPoint start = {currentPos, currentHeading};
    
    for (int i = 0; i < waypoints.size(); i++) {
        PathPoint end = waypoints[i];
        
        // Si no se especificó una orientación para el punto final, calcularla
        double endHeading = end.heading_deg;
        if (endHeading < 0) {
            // Calcular orientación basada en la dirección del segmento
            if (i < waypoints.size() - 1) {
                // Orientación hacia el siguiente waypoint
                double dx = waypoints[i+1].position.Xcord - end.position.Xcord;
                double dy = waypoints[i+1].position.Ycord - end.position.Ycord;
                endHeading = atan2(dy, dx) * 180.0 / M_PI;
            } else {
                // Para el último punto, mantener la dirección del segmento anterior
                endHeading = start.heading_deg;
            }
        }
        
        // Generar el segmento de trayectoria Bezier
        std::vector<Point> segment = generateBezierPath(
            start.position, 
            start.heading_deg, 
            end.position, 
            endHeading
        );
        
        // Agregar los puntos a la trayectoria completa (excepto el último punto para evitar duplicados)
        if (i > 0) {
            segment.erase(segment.begin()); // Eliminar el primer punto para evitar duplicados
        }
        fullPath.insert(fullPath.end(), segment.begin(), segment.end());
        
        // El punto final se convierte en el punto de inicio para el siguiente segmento
        start = {end.position, endHeading};
    }
    
    return fullPath;
}

// Función para parsear una lista de coordenadas separadas por comas
std::vector<PathPoint> parseWaypoints(const char* waypointsStr) {
    std::vector<PathPoint> waypoints;
    
    // Cadena temporal para trabajar con la entrada
    std::string input(waypointsStr);
    std::stringstream ss(input);
    std::string token;
    
    // Parsear cada par de coordenadas
    while (std::getline(ss, token, ',')) {
        std::stringstream coordStream(token);
        double x, y;
        char delimiter;
        
        if (coordStream >> x >> delimiter >> y) {
            waypoints.push_back({x, y});
        }
    }
    
    return waypoints;
}

// Función principal para llamar al control de trayectoria
void followPath(Point destination) {
    std::vector<PathPoint> waypoints = {{destination}};
    path = generatePathWithWaypoints(waypoints);
    purePursuit();
}

// Función para seguir una trayectoria con destino y puntos intermedios
void followPathWithWaypoints(Point destination, const char* waypointsStr = nullptr) {
    std::vector<PathPoint> waypoints;
    
    // Añadir puntos intermedios si se especificaron
    if (waypointsStr != nullptr) {
        waypoints = parseWaypoints(waypointsStr);
    }
    
    // Añadir el punto de destino al final
    waypoints.push_back({destination});
    
    // Generar la trayectoria completa
    path = generatePathWithWaypoints(waypoints);
    
    // Seguir la trayectoria
    purePursuit();
}

void purePursuit() 
{
    bool iscompleted = false;
    while (!iscompleted) {
      // Obtener posición y heading
      double x = GPS.xPosition();
      double y = GPS.yPosition();
      double heading = degToRad(GPS.heading());
  
      Point current = {x, y};
      Point target = getLookahead(current);
  
      // Calcular ángulo hacia el objetivo
      double dx = target.Xcord - x;
      double dy = target.Ycord - y;
      double target_angle = atan2(dy, dx);
  
      double angle_error = normalizeAngle(target_angle - heading);
  
      // Control de motores
      double turn = turn_k * angle_error;
      double left_speed = base_speed - turn * 50;
      double right_speed = base_speed + turn * 50;

      double leftSpeedinVolts = (left_speed/100) * 12;
      double rightSpeedinVolts = (right_speed/100) * 12;

      // fprintf(fp, "\r Left %.2f, Right %.2f\n", leftSpeedinVolts, rightSpeedinVolts);
      Chassis.DriveL.spin(fwd, left_speed, pct);
      Chassis.DriveR.spin(fwd, right_speed, pct);
  
      // Fin si estamos muy cerca del último punto
      if (distanceTo(current, path.back()) < 50) {
        Chassis.drive_with_voltage(0,0);
        iscompleted = true;
        break;
      }
  
      wait(20, msec);
    }
}

// Ejemplos de uso:
// Para ir a un punto específico:
// followPath({120, 120});
//
// Para ir a un punto con waypoints intermedios:
// followPathWithWaypoints({120, 120}, "60 60, 90 90, 80 120");