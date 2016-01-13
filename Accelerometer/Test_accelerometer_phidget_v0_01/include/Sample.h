#ifndef SAMPLE_H
#define SAMPLE_H

#include <stdio.h>

class Vec3{
    public:
        Vec3(){};
        Vec3(double _x, double _y, double _z){x=_x; y=_y; z=_z;}
        virtual ~Vec3(){};
        double x;
        double y;
        double z;
};

class Sample
{
    public:

        Sample();
        Sample(size_t _num_frame, double _timestamp, Vec3 _acceleration, Vec3 _angularRate, Vec3 _magneticField, Vec3 _velocity, Vec3 _position);
        virtual ~Sample();

        size_t num_frame;
        double timestamp;
        Vec3 acceleration;
        Vec3 angularRate;
        Vec3 magneticField;
        Vec3 velocity;
        Vec3 position;

        void displaySample();

    protected:
    private:
};

#endif // SAMPLE_H
