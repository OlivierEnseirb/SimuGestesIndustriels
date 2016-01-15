#ifndef IMU_MATHS_H
#define IMU_MATHS_H

#include <stdio.h>
#include <cmath>
#include "Sample.h"

using namespace std;

#define MATHS_PI ((DATA_TYPE)3.1415926)

enum DATA_FORMAT {CSV, TRC};

const Sample defaultSample = Sample(0, 0.0, Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec4(1.0, 1.0, 1.0, 1.0));

class IMU_maths
{
    public:

        IMU_maths();
        virtual ~IMU_maths();

        void accelerationIntegration1Axe(const double acceleration, double& velocity, double& position, const double initial_position, const double initial_velocity, const double deltaTime);
        void accelerationIntegration3Axes(const Vec3 acceleration, Vec3& velocity, Vec3& position, const Vec3 initial_position, const Vec3 initial_velocity, const double deltaTime);
        void accelerationIntegration3Axes(Sample& new_sample, Sample& previous_sample);

        void QuaternionToEuler(const Vec4 q, Vec3& v);
        void EulerToQuaternion(const Vec3 v, Vec4& q);
        void NormalizeQuaternion(Vec4& q);
        DATA_TYPE NormQuaternion(const Vec4 q);

        void writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data, unsigned char data_to_display);

       /** Write the header on the specified file
        * @param _file - the file where to write the header.
        * @param _df - format in which data will be written
        */
        void writeHeaderInFile(FILE* _file, DATA_FORMAT _df, unsigned char data_to_display);

    protected:
    private:
        

};
#endif // IMU_MATHS_H
