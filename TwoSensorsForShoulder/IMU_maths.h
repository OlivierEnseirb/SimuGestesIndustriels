/***************************************
 * Author : Olivier Hartmann (dec. 2015)
 * This object contains all mathematical functions
 ***************************************/

#ifndef IMU_MATHS_H
#define IMU_MATHS_H

#include <stdio.h>
#include <cmath>
#include "Sample.h"

using namespace std;

#define MATHS_PI ((DATA_TYPE)3.1415926)
#define MAT_SIZE 3

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

        void QuaternionToEuler(const Vec4 q, Vec3& v); // convert a quaternion to a euler vector
        void EulerToQuaternion(const Vec3 v, Vec4& q); // convert euler vector to a quaternion
		void QuaternionToOrthogonalMatrix(const Vec4 q_in, DATA_TYPE matrix[MAT_SIZE][MAT_SIZE]); // convert quaternion to an orthogonal rotational matrix
		void OrthogonalMatrixToQuaternion(const  DATA_TYPE matrix[MAT_SIZE][MAT_SIZE], Vec4& q);  // convert an orthogonal rotational matrix to a quaternion

        void NormalizeQuaternion(Vec4& q); // normalize the matrix (to have |q| = 1)
        DATA_TYPE NormQuaternion(const Vec4 q); // return the norm of the quaternion q

		/* TO DO 
		 * change these two functions to write data coming from several sensors
		 */
        void writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data, unsigned char data_to_write); // write the data from the sample "data" into the file "file" according to the selected data to write in "data_to_write"
        void writeHeaderInFile(FILE* _file, DATA_FORMAT _df, unsigned char data_to_write);// write the data from the sample "data" into the file "file" according to the selected data to write in "data_to_write"

		void displayMatrix(const DATA_TYPE matrix[MAT_SIZE][MAT_SIZE]); // display the given matrix
		void addMatrix(const DATA_TYPE matrix1[MAT_SIZE][MAT_SIZE], const DATA_TYPE matrix2[MAT_SIZE][MAT_SIZE], DATA_TYPE result[MAT_SIZE][MAT_SIZE]); // result = mat1 + mat2
		void subMatrix(const DATA_TYPE matrix1[MAT_SIZE][MAT_SIZE], const DATA_TYPE matrix2[MAT_SIZE][MAT_SIZE], DATA_TYPE result[MAT_SIZE][MAT_SIZE]); // result = mat1 - mat2
		void mulMatrix(const DATA_TYPE matrix1[MAT_SIZE][MAT_SIZE], const DATA_TYPE matrix2[MAT_SIZE][MAT_SIZE], DATA_TYPE result[MAT_SIZE][MAT_SIZE]); // result = mat1 * mat2
		void mulMatrix(const DATA_TYPE matrix1[MAT_SIZE][MAT_SIZE], const DATA_TYPE real, DATA_TYPE result[MAT_SIZE][MAT_SIZE]); //result = real * matrix1
		void traMatrix(const DATA_TYPE matrix[MAT_SIZE][MAT_SIZE], DATA_TYPE result[MAT_SIZE][MAT_SIZE]); // result = transpose(matrix)
		void copMatrix(const DATA_TYPE matrix1[MAT_SIZE][MAT_SIZE], DATA_TYPE matrix2[MAT_SIZE][MAT_SIZE]); // mat2 = mat1

    protected:
    private:
        

};
#endif // IMU_MATHS_H
