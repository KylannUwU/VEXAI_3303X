#include "robot-config.h"
#include "track.h"
#include "vex.h"

using namespace std;
using namespace vex;


int quality(int currentDevice)
{
    vex::gps* device = nullptr;

    if (currentDevice == 1)
        device = &LGPS;
    else if(currentDevice == 2)
        device = &RGPS;

    else
        return 0;

    // Verificamos que el puntero no sea nulo antes de usarlo
    if (device != nullptr)
    {
        double quality = device->quality();
        fprintf(fp, "\r Qual is %.2f \n", quality);
        if(quality > 90)
            return 1;
        else
            return 0;
    }
    else
    return 0;
}

int trackingMain()
{
    while(1)
    {
        double
            x1 = LGPS.xPosition(vex::distanceUnits::mm),
            x2 = RGPS.xPosition(vex::distanceUnits::mm),
            y1 = LGPS.yPosition(vex::distanceUnits::mm),
            y2 = RGPS.yPosition(vex::distanceUnits::mm),
            lQual = quality(1),
            rQual = quality(2),
            tQual = lQual + rQual, 
            xPos = ((x1 * lQual) + (x2 * rQual)) / tQual,
            yPos = ((y1 * lQual) + (y2 * rQual)) / tQual;

        fprintf(fp,"\r newGPS val X: %.2f is %.2f + %.2f Y: %.2f is %.2f + %.2f \n", xPos, x1, x2, yPos, y1, y2); 
        // fprint(dp)

        wait(20,msec);
    }
} 