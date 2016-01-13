#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdio.h>

#ifndef DATA_TYPE
    #define DATA_TYPE  double
#endif // DATA_TYPE

class Quaternion{
    public:
        Quaternion();
        Quaternion(DATA_TYPE w, DATA_TYPE x, DATA_TYPE y, DATA_TYPE z);
        //Quaternion(Quaternion& q);
        virtual ~Quaternion();
        DATA_TYPE data[4];

        const size_t quat_w_pos = 0;
        const size_t quat_x_pos = 1;
        const size_t quat_y_pos = 2;
        const size_t quat_z_pos = 3;

        DATA_TYPE quat_w();
        DATA_TYPE quat_x();
        DATA_TYPE quat_y();
        DATA_TYPE quat_z();

        void quat_w(const DATA_TYPE val);
        void quat_x(const DATA_TYPE val);
        void quat_y(const DATA_TYPE val);
        void quat_z(const DATA_TYPE val);

        const DATA_TYPE operator[](int index) const;
        DATA_TYPE& operator[](int index);
        Quaternion& operator=(Quaternion& q);

        void display();
};
#endif // QUATERNION_H
