#ifndef PATHCONTROL_H
#define PATHCONTROL_H

#include <vector>
#include <string>

#include "field.h"


// Variables globales de configuración
extern double lookahead_dist;
extern double base_speed;
extern double turn_k;

// Funciones de utilidad
double distanceTo(Point a, Point b);
double degToRad(double deg);
double normalizeAngle(double angle);

// Generación de caminos

// Posición y seguimiento

Point getLookahead(Point current);

// Control de movimiento
void purePursuit();
void followPath(Point destination);
void followPathWithWaypoints(Point destination, const char* waypointsStr);

#endif // PATHCONTROL_H
