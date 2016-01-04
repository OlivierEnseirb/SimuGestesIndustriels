#include "IMU_maths.h"

IMU_maths::IMU_maths()
{
    //ctor
}

IMU_maths::~IMU_maths()
{
    //dtor
}

void IMU_maths::accelerationIntegration1Axe(const double acceleration, double& velocity, double& position, const double initial_position, const double initial_velocity, const double deltaTime)
{
    double instant_velocity = acceleration * deltaTime;
    velocity = instant_velocity + initial_velocity;
    position = instant_velocity * deltaTime /2 + initial_velocity * deltaTime + initial_position;
}

void IMU_maths::accelerationIntegration3Axes(const Vec3 acceleration, Vec3& velocity, Vec3& position, const Vec3 initial_position, const Vec3 initial_velocity, const double deltaTime)
{
    accelerationIntegration1Axe(acceleration.x, velocity.x, position.x, initial_position.x, initial_velocity.x, deltaTime);
    accelerationIntegration1Axe(acceleration.y, velocity.y, position.y, initial_position.y, initial_velocity.y, deltaTime);
    accelerationIntegration1Axe(acceleration.z-1, velocity.z, position.z, initial_position.z, initial_velocity.z, deltaTime);
}

void IMU_maths::accelerationIntegration3Axes(Sample& new_sample, Sample& previous_sample)
{
    double deltaTime = new_sample.timestamp - previous_sample.timestamp;
    accelerationIntegration3Axes(new_sample.acceleration, new_sample.velocity, new_sample.position, previous_sample.position, previous_sample.velocity, deltaTime);
}

void IMU_maths::writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data)
{
    if(file != NULL)
        switch(_df)
        {
        case CSV :
                // Timestamp, then Accel X,Y,Z, then Ang Rate X,Y,Z then Mag Field X,Y,Z then velocity the position
                fprintf(file, "%f,", data.timestamp);
                fprintf(file, "%6f,%6f,%6f,", data.acceleration.x, data.acceleration.y, data.acceleration.z);
                fprintf(file, "%6f,%6f,%6f,", data.angularRate.x, data.angularRate.y, data.angularRate.z);
                // Due to time for internal calibration, the compass sometimes returns the large constant PUNK_DBL
                if (data.magneticField.x == PUNK_DBL) {
                    fprintf(file, "%6f,%6f,%6f,", 0.0, 0.0, 0.0);
                } else {
                    fprintf(file, "%6f,%6f,%6f,", data.magneticField.x, data.magneticField.y, data.magneticField.z);
                }

                fprintf(file, "%6f,%6f,%6f,", data.velocity.x, data.velocity.y, data.velocity.z);
                fprintf(file, "%6f,%6f,%6f\n", data.position.x, data.position.y, data.position.z);
                fflush(file);
            break;
        case TRC:
                fprintf(file, "%d\t%6f\t", data.num_frame, data.timestamp);
                fprintf(file, "%6f\t%6f\t%6f\n", data.position.x, data.position.y, data.position.z);
            break;
        }
}

void IMU_maths::writeHeaderInFile(FILE* _file, DATA_FORMAT _df)
{
    if(_file != NULL)
        switch(_df)
        {
        case CSV :
            fprintf(_file, "Time,Accel_X,Accel_Y,Accel_Z,Ang_X,Ang_Y,Ang_Z,Mag_X,Mag_Y,Mag_Z,Vel_X,Vel_Y,Vel_Z,Pos_X,Pos_Y,Pos_Z\n");
            break;

        case TRC:
            fprintf(_file, "PathFileType\t4\t(X/Y/Z)\n");
            fprintf(_file, "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n");
            fprintf(_file, "16\t120\t121\t3\tm\t120\t1\t121\n");
            fprintf(_file, "Frame#\tTime\tr_radius_styloid\t\t\n");
            fprintf(_file, "\t\tX1\tY1\tZ1\n");

            break;
        }
}

void IMU_maths::displaySample(Sample& _samp)
{
    printf("Sample %3d, timestamp %f s :\n", _samp.num_frame, _samp.timestamp);
    printf("\t acc x = %f, acc y = %f, acc z = %f\n", _samp.acceleration.x, _samp.acceleration.y, _samp.acceleration.z);
    printf("\t ang x = %f, ang y = %f, ang z = %f\n", _samp.angularRate.x, _samp.angularRate.y, _samp.angularRate.z);
    printf("\t mag x = %f, mag y = %f, mag z = %f\n", _samp.magneticField.x, _samp.magneticField.y, _samp.magneticField.z);
    printf("\t vel x = %f, vel y = %f, vel z = %f\n", _samp.velocity.x, _samp.velocity.y, _samp.velocity.z);
    printf("\t pos x = %f, pos y = %f, pos z = %f\n", _samp.position.x, _samp.position.y, _samp.position.z);
}
