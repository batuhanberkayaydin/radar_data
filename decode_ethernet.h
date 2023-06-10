#ifndef DECODE_ETHERNET_H
#define DECODE_ETHERNET_H

// struct radarDatas{
//     int objectId;
//     int x_coord;
//     int y_coord;
//     int z_coord;
// };

void initRadarData();
int getRadarData();
char *printSMSObject_V3_1(int index);
// radarDatas *rData;

// int getObjectId(int x);

// extern int objectId;
// float x_coord;


#endif // DECODE_ETHERNET_H