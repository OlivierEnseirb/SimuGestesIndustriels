#include "Sample.h"

Sample::Sample()
{
    //ctor
}

Sample::~Sample()
{
    //dtor
}
Sample::Sample(size_t _num_frame, double _timestamp, Vec3 _acceleration, Vec3 _angularRate, Vec3 _magneticField, Vec3 _velocity, Vec3 _position)
{
    num_frame = _num_frame;
    timestamp = _timestamp;
    acceleration = _acceleration;
    angularRate = _angularRate;
    magneticField = _magneticField;
    velocity = _velocity;
    position = _position;
}

void Sample::displaySample()
{
    printf("Sample %3d, timestamp %f s :\n", num_frame, timestamp);
    printf("\t acc x = %f, acc y = %f, acc z = %f\n", acceleration.x, acceleration.y, acceleration.z);
    printf("\t ang x = %f, ang y = %f, ang z = %f\n", angularRate.x, angularRate.y, angularRate.z);
    printf("\t mag x = %f, mag y = %f, mag z = %f\n", magneticField.x, magneticField.y, magneticField.z);
    printf("\t vel x = %f, vel y = %f, vel z = %f\n", velocity.x, velocity.y, velocity.z);
    printf("\t pos x = %f, pos y = %f, pos z = %f\n", position.x, position.y, position.z);
}
