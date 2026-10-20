#ifndef MESHPOCKET_V2_GPS_TEST_H
#define MESHPOCKET_V2_GPS_TEST_H

#include <Arduino_GFX_Library.h>

void gpsBegin();
void gpsPrepareDisplay(Arduino_GFX &display, const char *footer);
void gpsPoll();
void gpsDraw(Arduino_GFX &display);
void gpsEnd();

#endif
