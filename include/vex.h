/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//

#include <initializer_list> 
#include <algorithm>
#include <iostream>
#include <cstdarg>
#include <utility>
#include <vector>
#include <cmath>
#include <tuple>
#include <set>

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "v5.h"
#include "v5_vcs.h"

#include "ai_jetson.h"
#include "ai_robot_link.h"

#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"


#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

extern ai::jetson      jetson_comms;
extern ai::robot_link  link;

extern int dashboardTask( void );
