#include "Quaternion.h"

Quaternion::Quaternion()
{

}

Quaternion::~Quaternion()
{

}

Quaternion::Quaternion(DATA_TYPE w, DATA_TYPE x, DATA_TYPE y, DATA_TYPE z)
{
    data[quat_w_pos] = w;
    data[quat_x_pos] = x;
    data[quat_y_pos] = y;
    data[quat_z_pos] = z;
}

/*Quaternion::Quaternion(Quaternion& q)
{
    quat_w(q.quat_w());
    quat_x(q.quat_x());
    quat_y(q.quat_y());
    quat_z(q.quat_z());
}*/

DATA_TYPE Quaternion::quat_w()
{
    return data[quat_w_pos];
}

DATA_TYPE Quaternion::quat_x()
{
    return data[quat_x_pos];
}

DATA_TYPE Quaternion::quat_y()
{
    return data[quat_y_pos];
}

DATA_TYPE Quaternion::quat_z()
{
    return data[quat_z_pos];
}

void Quaternion::quat_w(const DATA_TYPE val)
{
    data[quat_w_pos] = val;
}

void Quaternion::quat_x(const DATA_TYPE val)
{
    data[quat_x_pos] = val;
}

void Quaternion::quat_y(const DATA_TYPE val)
{
    data[quat_y_pos] = val;
}

void Quaternion::quat_z(const DATA_TYPE val)
{
    data[quat_z_pos] = val;
}

const DATA_TYPE Quaternion::operator[](int index) const
{
    return data[index];
}

DATA_TYPE& Quaternion::operator[](int index)
{
    return *(data + index);
}

Quaternion& Quaternion::operator=(Quaternion& q)
{
    quat_w(q.quat_w());
    quat_x(q.quat_x());
    quat_y(q.quat_y());
    quat_z(q.quat_z());
    return *this;
}

void Quaternion::display()
{
    printf("qua w = %f, qua x = %f, qua y = %f, qua z = %f\n", quat_w(), quat_x(), quat_y(), quat_z());
}

