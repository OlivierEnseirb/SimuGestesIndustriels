#ifndef IMU_MATHS_H
#define IMU_MATHS_H

#include <stdio.h>
#include <phidget21.h>
#include "Sample.h"

enum DATA_FORMAT {CSV, TRC};

class IMU_maths
{
    public:

      /*  struct Vec3{double x;
                    double y;
                    double z;};
        struct Sample{  size_t num_frame;
                        double timestamp;
                        Vec3 acceleration;
                        Vec3 angularRate;
                        Vec3 magneticField;
                        Vec3 velocity;
                        Vec3 position;};
*/
        IMU_maths();
        virtual ~IMU_maths();

        void accelerationIntegration1Axe(const double acceleration, double& velocity, double& position, const double initial_position, const double initial_velocity, const double deltaTime);
        void accelerationIntegration3Axes(const Vec3 acceleration, Vec3& velocity, Vec3& position, const Vec3 initial_position, const Vec3 initial_velocity, const double deltaTime);
        void accelerationIntegration3Axes(Sample& new_sample, Sample& previous_sample);

        void writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data);

       /** Write the header on the specified file
        * @param _file - the file where to write the header.
        * @param _df - format in which data will be written
        */
        void writeHeaderInFile(FILE* _file, DATA_FORMAT _df);

       // void displaySample(Sample& _samp);
        Sample getDefaultSample();
    protected:
    private:
        const Sample * defaultSample = new Sample(0, 0.0, Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));

};

#endif // IMU_MATHS_H
