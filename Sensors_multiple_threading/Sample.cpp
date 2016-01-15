#include "Sample.h"

Sample::Sample()
{
    //ctor
}

Sample::~Sample()
{
    //dtor
}

Sample::Sample(int _num_frame, double _timestamp, Vec3 _acceleration, Vec3 _angularRate, Vec3 _magneticField, Vec3 _velocity, Vec3 _position, Vec4 _quaternion)
{
    num_frame = _num_frame;
    timestamp = _timestamp;
    acceleration = _acceleration;
    angularRate = _angularRate;
    magneticField = _magneticField;
    velocity = _velocity;
    position = _position;
    quaternion = _quaternion;
}

Sample::Sample(Sample& _s)
{
    num_frame = _s.num_frame;
    timestamp = _s.timestamp;
    acceleration = _s.acceleration;
    angularRate = _s.angularRate;
    magneticField = _s.magneticField;
    velocity = _s.velocity;
    position = _s.position;
    quaternion = _s.quaternion;
}

Sample::Sample(const Sample& _s)
{
    num_frame = _s.num_frame;
    timestamp = _s.timestamp;
    acceleration = _s.acceleration;
    angularRate = _s.angularRate;
    magneticField = _s.magneticField;
    velocity = _s.velocity;
    position = _s.position;
    quaternion = _s.quaternion;
}

void Sample::displaySample()
{
    printf("Sample %3d, timestamp %1f s :\n", num_frame, timestamp);
    printf("\t acc x = %1f, acc y = %1f, acc z = %1f\n", acceleration.x, acceleration.y, acceleration.z);
    printf("\t ang x = %1f, ang y = %1f, ang z = %1f\n", angularRate.x, angularRate.y, angularRate.z);
    printf("\t mag x = %1f, mag y = %1f, mag z = %1f\n", magneticField.x, magneticField.y, magneticField.z);
    printf("\t vel x = %1f, vel y = %1f, vel z = %1f\n", velocity.x, velocity.y, velocity.z);
    printf("\t pos x = %1f, pos y = %1f, pos z = %1f\n", position.x, position.y, position.z);
    printf("\t qua a = %1f, qua b = %1f, qua c = %1f, qua d = %1f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    printf("\t eul x = %1f, eul y = %1f, eul z = %1f\n", euler.x, euler.y, euler.z);
}

void Sample::displaySample(unsigned char data_to_display)
{
    if(data_to_display)
        printf("Sample %3d, timestamp %1f s :\n", num_frame, timestamp);
    if(data_to_display & ENABLE_ACCELERETION)
        printf("\t acc x = %1f, acc y = %1f, acc z = %1f\n", acceleration.x, acceleration.y, acceleration.z);
    if(data_to_display & ENABLE_ANGULAR_RATE)
        printf("\t ang x = %1f, ang y = %1f, ang z = %1f\n", angularRate.x, angularRate.y, angularRate.z);
    if(data_to_display & ENABLE_MAGNETIC_FIELD)
        printf("\t mag x = %1f, mag y = %1f, mag z = %1f\n", magneticField.x, magneticField.y, magneticField.z);
    if(data_to_display & ENABLE_VELOCITY)
        printf("\t vel x = %1f, vel y = %1f, vel z = %1f\n", velocity.x, velocity.y, velocity.z);
    if(data_to_display & ENABLE_POSITION)
        printf("\t pos x = %1f, pos y = %1f, pos z = %1f\n", position.x, position.y, position.z);
    if(data_to_display & ENABLE_QUATERNION)
        printf("\t qua a = %1f, qua b = %1f, qua c = %1f, qua d = %1f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    if(data_to_display & ENABLE_EULER)
        printf("\t eul x = %1f, eul y = %1f, eul z = %1f\n", euler.x, euler.y, euler.z);
	if(data_to_display)
		printf("\n");
}

