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

void IMU_maths::writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data, unsigned char data_to_display)
{
    if(file != NULL && data_to_display != 0x00)
        switch(_df)
        {
        case CSV :
                // Timestamp, then Accel X,Y,Z, then Ang Rate X,Y,Z then Mag Field X,Y,Z then velocity the position
                fprintf(file, "%f", data.timestamp);
                if(data_to_display & DISPLAY_ACCELERETION)
                    fprintf(file, ",%6f,%6f,%6f", data.acceleration.x, data.acceleration.y, data.acceleration.z);
                if(data_to_display & DISPLAY_ANGULAR_RATE)
                    fprintf(file, ",%6f,%6f,%6f", data.angularRate.x, data.angularRate.y, data.angularRate.z);
                if(data_to_display & DISPLAY_MAGNETIC_FIELD)
                {
                    // Due to time for internal calibration, the compass sometimes returns the large constant PUNK_DBL
                    if (data.magneticField.x == PUNK_DBL)
                    {
                        fprintf(file, ",%6f,%6f,%6f", 0.0, 0.0, 0.0);
                    }
                    else
                    {
                        fprintf(file, ",%6f,%6f,%6f", data.magneticField.x, data.magneticField.y, data.magneticField.z);
                    }
                }
                if(data_to_display & DISPLAY_VELOCITY)
                    fprintf(file, ",%6f,%6f,%6f", data.velocity.x, data.velocity.y, data.velocity.z);
                if(data_to_display & DISPLAY_POSITION)
                    fprintf(file, ",%6f,%6f,%6f", data.position.x, data.position.y, data.position.z);
                if(data_to_display & DISPLAY_QUATERNION)
                    fprintf(file, ",%6f,%6f,%6f,%6f", data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z);
                if(data_to_display & DISPLAY_EULER)
                    fprintf(file, ",%6f,%6f,%6f", data.euler.x, data.euler.y, data.euler.z);
                fprintf(file, "\n");
            break;
        case TRC:
                fprintf(file, "%d\t%6f\t", data.num_frame, data.timestamp);
                fprintf(file, "%6f\t%6f\t%6f\n", data.position.x, data.position.y, data.position.z);
            break;
        }
        fflush(file);
}

void IMU_maths::writeHeaderInFile(FILE* _file, DATA_FORMAT _df, unsigned char data_to_display)
{
    if(_file != NULL)
        switch(_df)
        {
        case CSV :
            fprintf(_file, "Time");

            if(data_to_display & DISPLAY_ACCELERETION)
                fprintf(_file, ",Accel_X,Accel_Y,Accel_Z");
            if(data_to_display & DISPLAY_ANGULAR_RATE)
                fprintf(_file, ",Ang_X,Ang_Y,Ang_Z");
            if(data_to_display & DISPLAY_MAGNETIC_FIELD)
                fprintf(_file, ",Mag_X,Mag_Y,Mag_Z");
            if(data_to_display & DISPLAY_VELOCITY)
                fprintf(_file, ",Vel_X,Vel_Y,Vel_Z");
            if(data_to_display & DISPLAY_POSITION)
                fprintf(_file, ",Pos_X,Pos_Y,Pos_Z");
            if(data_to_display & DISPLAY_QUATERNION)
                fprintf(_file, ",Qua_A,Qua_B,Qua_C,Qua_D");
            if(data_to_display & DISPLAY_EULER)
                fprintf(_file, ",Eul_A,Eul_B,Eul_C");

            fprintf(_file, "\n");
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

Sample IMU_maths::getDefaultSample()
{
    return *defaultSample;
}

void IMU_maths::QuaternionToEuler(const Vec4 q, Vec3& v)
{
  DATA_TYPE pole = MATHS_PI/((DATA_TYPE)2.0) - (DATA_TYPE)0.05; // fix roll near poles with this tolerance

  v.y = asin((DATA_TYPE)2.0 * (q.w * q.y - q.x * q.z));

  if ((v.y < pole) && (v.y > -pole))
	  v.x = atan2((DATA_TYPE)2.0 * (q.y * q.z + q.w * q.x),
                    (DATA_TYPE)1.0 - (DATA_TYPE)2.0 * (q.x * q.x + q.y * q.y));

  v.z = atan2((DATA_TYPE)2.0 * (q.x * q.y + q.w * q.z),
                    (DATA_TYPE)1.0 - (DATA_TYPE)2.0 * (q.y * q.y + q.z * q.z));
}

void IMU_maths::EulerToQuaternion(const Vec3 v, Vec4& q)
{
  float cosX2 = cos(v.x / (DATA_TYPE)2.0);
  float sinX2 = sin(v.x / (DATA_TYPE)2.0);
  float cosY2 = cos(v.y / (DATA_TYPE)2.0);
  float sinY2 = sin(v.y / (DATA_TYPE)2.0);
  float cosZ2 = cos(v.z / (DATA_TYPE)2.0);
  float sinZ2 = sin(v.z / (DATA_TYPE)2.0);

  q.w = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
  q.x = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
  q.y = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
  q.z = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
  NormalizeQuaternion(q);
}

void IMU_maths::NormalizeQuaternion(Vec4& q)
{
    DATA_TYPE length = NormQuaternion(q);
    if (length == 0)
        return;
    q.w /= length;
    q.x /= length;
    q.y /= length;
    q.z /= length;
}

DATA_TYPE IMU_maths::NormQuaternion(const Vec4 q)
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

