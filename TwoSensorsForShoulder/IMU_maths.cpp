#include "IMU_maths.h"

IMU_maths::IMU_maths()
{
    //ctor
}

IMU_maths::~IMU_maths()
{
    //dtor
}

void IMU_maths::accelerationIntegration1Axe(const DATA_TYPE acceleration, DATA_TYPE& velocity, DATA_TYPE& position, const DATA_TYPE initial_position, const DATA_TYPE initial_velocity, const DATA_TYPE deltaTime)
{
    DATA_TYPE instant_velocity = acceleration * deltaTime;
    velocity = instant_velocity + initial_velocity;
    position = instant_velocity * deltaTime /2 + initial_velocity * deltaTime + initial_position;
}

void IMU_maths::accelerationIntegration3Axes(const Vec3 acceleration, Vec3& velocity, Vec3& position, const Vec3 initial_position, const Vec3 initial_velocity, const DATA_TYPE deltaTime)
{
    accelerationIntegration1Axe(acceleration.x, velocity.x, position.x, initial_position.x, initial_velocity.x, deltaTime);
    accelerationIntegration1Axe(acceleration.y, velocity.y, position.y, initial_position.y, initial_velocity.y, deltaTime);
    accelerationIntegration1Axe(acceleration.z-1, velocity.z, position.z, initial_position.z, initial_velocity.z, deltaTime);
}

void IMU_maths::accelerationIntegration3Axes(Sample& new_sample, Sample& previous_sample)
{
    DATA_TYPE deltaTime = new_sample.timestamp - previous_sample.timestamp;
    accelerationIntegration3Axes(new_sample.acceleration, new_sample.velocity, new_sample.position, previous_sample.position, previous_sample.velocity, deltaTime);
}

void IMU_maths::writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data, unsigned char data_to_write)
{
    if(file != NULL)
        switch(_df)
        {
        case CSV :
				if(data_to_write)
					// Timestamp, then Accel X,Y,Z, then Ang Rate X,Y,Z then Mag Field X,Y,Z then velocity the position
				    fprintf(file, "%f", data.timestamp);
                if(data_to_write & ENABLE_ACCELERETION)
                    fprintf(file, ",%6f,%6f,%6f", data.acceleration.x, data.acceleration.y, data.acceleration.z);
                if(data_to_write & ENABLE_ANGULAR_RATE)
                    fprintf(file, ",%6f,%6f,%6f", data.angularRate.x, data.angularRate.y, data.angularRate.z);
                if(data_to_write & ENABLE_MAGNETIC_FIELD)
                {
                    // Due to time for internal calibration, the compass sometimes returns the large constant PUNK_DBL
                    /*if (data.magneticField.x == PUNK_DBL)
                    {
                        fprintf(file, ",%6f,%6f,%6f", 0.0, 0.0, 0.0);
                    }
                    else
                    {*/
                        fprintf(file, ",%6f,%6f,%6f", data.magneticField.x, data.magneticField.y, data.magneticField.z);
                    //}
                }
                if(data_to_write & ENABLE_VELOCITY)
                    fprintf(file, ",%6f,%6f,%6f", data.velocity.x, data.velocity.y, data.velocity.z);
                if(data_to_write & ENABLE_POSITION)
                    fprintf(file, ",%6f,%6f,%6f", data.position.x, data.position.y, data.position.z);
                if(data_to_write & ENABLE_QUATERNION)
                    fprintf(file, ",%6f,%6f,%6f,%6f", data.quaternion.w, data.quaternion.x, data.quaternion.y, data.quaternion.z);
                if(data_to_write & ENABLE_EULER)
                    fprintf(file, ",%6f,%6f,%6f", data.euler.x, data.euler.y, data.euler.z);
				if (data_to_write)
					fprintf(file, "\n");
            break;
        case TRC:
                fprintf(file, "%d\t%6f\t", data.num_frame, data.timestamp);
                fprintf(file, "%6f\t%6f\t%6f\n", data.position.x, data.position.y, data.position.z);
            break;
        }
        fflush(file);
}

void IMU_maths::writeHeaderInFile(FILE* _file, DATA_FORMAT _df, unsigned char data_to_write)
{
    if(_file != NULL)
        switch(_df)
        {
        case CSV :
			if (data_to_write)
				fprintf(_file, "Time");
            if(data_to_write & ENABLE_ACCELERETION)
                fprintf(_file, ",Accel_X,Accel_Y,Accel_Z");
            if(data_to_write & ENABLE_ANGULAR_RATE)
                fprintf(_file, ",Ang_X,Ang_Y,Ang_Z");
            if(data_to_write & ENABLE_MAGNETIC_FIELD)
                fprintf(_file, ",Mag_X,Mag_Y,Mag_Z");
            if(data_to_write & ENABLE_VELOCITY)
                fprintf(_file, ",Vel_X,Vel_Y,Vel_Z");
            if(data_to_write & ENABLE_POSITION)
                fprintf(_file, ",Pos_X,Pos_Y,Pos_Z");
            if(data_to_write & ENABLE_QUATERNION)
                fprintf(_file, ",Qua_A,Qua_B,Qua_C,Qua_D");
            if(data_to_write & ENABLE_EULER)
                fprintf(_file, ",Eul_A,Eul_B,Eul_C");
			if(data_to_write)
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
  DATA_TYPE cosX2 = cos(v.x / (DATA_TYPE)2.0);
  DATA_TYPE sinX2 = sin(v.x / (DATA_TYPE)2.0);
  DATA_TYPE cosY2 = cos(v.y / (DATA_TYPE)2.0);
  DATA_TYPE sinY2 = sin(v.y / (DATA_TYPE)2.0);
  DATA_TYPE cosZ2 = cos(v.z / (DATA_TYPE)2.0);
  DATA_TYPE sinZ2 = sin(v.z / (DATA_TYPE)2.0);

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

void IMU_maths::QuaternionToOrthogonalMatrix(const Vec4 q, RotMat matrix)
{
	DATA_TYPE w2 = q.w * q.w;
	DATA_TYPE x2 = q.x * q.x;
	DATA_TYPE y2 = q.y * q.y;
	DATA_TYPE z2 = q.z * q.z;
	DATA_TYPE wx = q.w * q.x;
	DATA_TYPE xy = q.x * q.y;
	DATA_TYPE yz = q.y * q.z;
	DATA_TYPE wy = q.w * q.y;
	DATA_TYPE wz = q.w * q.z;
	DATA_TYPE xz = q.w * q.z;

	matrix[0][0] = 1 - 2*(y2 + z2);//w2 + x2 - y2 - z2;
	matrix[0][1] = 2 * xy - 2 * wz;
	matrix[0][2] = 2 * wy - 2 * xz;

	matrix[1][0] = 2 * wz + 2 * xy;
	matrix[1][1] = 1 - 2*(x2 + z2);//w2 - x2 + y2 - z2;
	matrix[1][2] = 2 * yz - 2 * wx;

	matrix[2][0] = 2 * xz - 2 * wy;
	matrix[2][1] = 2 * wx + 2 * yz;
	matrix[2][2] = 1 - 2*(x2 + y2);//w2 - x2 - y2 + z2;
}

void IMU_maths::OrthogonalMatrixToQuaternion(const RotMat matrix, Vec4& q)
{
	q.w = 0.5f * sqrtf((float)(1.0f + matrix[0][0] + matrix[1][1] + matrix[2][2]));
	q.x = 0.25f / q.w * (matrix[2][1] - matrix[1][2]);
	q.y = 0.25f / q.w * (matrix[0][2] - matrix[2][0]);
	q.z = 0.25f / q.w * (matrix[1][0] - matrix[0][1]);
}

void IMU_maths::displayMatrix(const RotMat matrix)
{
	for (int i = 0; i < MAT_SIZE; i++)
	{
		printf("{");
		for (int j = 0; j < MAT_SIZE; j++)
		{
			printf("%f", matrix[i][j]);
			if (j != (MAT_SIZE - 1))
				printf(" , ");
		}
		printf("}\n");
	}
}

void IMU_maths::addMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result)
{
	for (int c = 0; c < MAT_SIZE; c++)
		for (int d = 0; d < MAT_SIZE; d++)
			result[c][d] = matrix1[c][d] + matrix2[c][d];
}

void IMU_maths::subMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result)
{
	for (int c = 0; c < MAT_SIZE; c++)
		for (int d = 0; d < MAT_SIZE; d++)
			result[c][d] = matrix1[c][d] - matrix2[c][d];
}

void IMU_maths::mulMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result)
{
	for (int c = 0; c < MAT_SIZE; c++)
		for (int d = 0; d < MAT_SIZE; d++)
			result[c][d] = matrix1[c][0] * matrix2[0][d] + matrix1[c][1] * matrix2[1][d] + matrix1[c][2] * matrix2[2][d];
}

void IMU_maths::mulMatrix(const RotMat matrix1, const DATA_TYPE real, RotMat result)
{
	for (int c = 0; c < MAT_SIZE; c++)
		for (int d = 0; d < MAT_SIZE; d++)
			result[c][d] = real * matrix1[c][d];
}

void IMU_maths::traMatrix(const RotMat matrix, RotMat result)
{
	copMatrix(matrix, result);
	result[0][1] = matrix[1][0];
	result[0][2] = matrix[2][0];
	result[1][0] = matrix[0][1];
	result[1][2] = matrix[2][1];
	result[2][0] = matrix[0][2];
	result[2][1] = matrix[1][2];
}

void IMU_maths::copMatrix(const RotMat matrix1, RotMat matrix2)
{
	for (int c = 0; c < MAT_SIZE; c++)
		for (int d = 0; d < MAT_SIZE; d++)
			matrix2[c][d] = matrix1[c][d];
}