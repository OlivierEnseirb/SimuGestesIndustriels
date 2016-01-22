/***************************************
* Author : Olivier Hartmann (janv. 2015)
* This object is the base mathematical object containing all the possible physical data
***************************************/

#ifndef SAMPLE_H
#define SAMPLE_H

#include <stdio.h>

#define DATA_TYPE           double

#define ENABLE_NOTHING		   0x00
#define ENABLE_ACCELERETION    0x01
#define ENABLE_ANGULAR_RATE    0x02
#define ENABLE_MAGNETIC_FIELD  0x04
#define ENABLE_VELOCITY        0x08
#define ENABLE_POSITION        0x10
#define ENABLE_EULER           0x20
#define ENABLE_QUATERNION      0x40

using namespace std;

class Vec3{
    public:
        Vec3(){}
        Vec3(DATA_TYPE _x, DATA_TYPE _y, DATA_TYPE _z){x=_x; y=_y; z=_z;}
        virtual ~Vec3(){}
        DATA_TYPE x;
        DATA_TYPE y;
        DATA_TYPE z;
};

class Vec4{
    public:
        Vec4(){}
        Vec4(DATA_TYPE _w, DATA_TYPE _x, DATA_TYPE _y, DATA_TYPE _z){w=_w; x=_x; y=_y; z=_z;}
        virtual ~Vec4(){}
        DATA_TYPE w;
        DATA_TYPE x;
        DATA_TYPE y;
        DATA_TYPE z;
};


class Sample
{
    public:
        Sample();
        //Sample(Sample& _s);
        Sample(const Sample& _s);
        Sample(int _num_frame, double _timestamp, Vec3 _acceleration, Vec3 _angularRate, Vec3 _magneticField, Vec3 _velocity, Vec3 _position, Vec4 _quaternion);
        virtual ~Sample();

        int num_frame = 0;
        double timestamp = 0.0;
        Vec3 acceleration = Vec3(0.0,0.0,0.0);
        Vec3 angularRate = Vec3(0.0,0.0,0.0);
        Vec3 magneticField = Vec3(0.0,0.0,0.0);
        Vec3 velocity = Vec3(0.0,0.0,0.0);
        Vec3 position = Vec3(0.0,0.0,0.0);
        Vec4 quaternion = Vec4(1.0,0.0,0.0,0.0);
        Vec3 euler = Vec3(0.0, 0.0, 0.0);

        void displaySample(unsigned char data_to_display);
        void displaySample();

};

#endif // SAMPLE_H
