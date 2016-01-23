/***************************************
 * Author : Olivier Hartmann (dec. 2015)
 * This object contains all mathematical functions
 ***************************************/

#ifndef IMU_MATHS_H
#define IMU_MATHS_H

#include <stdio.h>
#include <cmath>
#include "Sample.h"

#define MATHS_PI ((DATA_TYPE)3.1415926)
#define MAT_SIZE 3

using namespace std;


typedef DATA_TYPE RotMat[MAT_SIZE][MAT_SIZE]; // definition of the rotation matrix type
enum DATA_FORMAT { CSV, TRC }; // available format in which data would be written in the file

const Sample defaultSample = Sample(0, 0.0, Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec4(1.0, 1.0, 1.0, 1.0));

class IMU_maths
{
    public:

        IMU_maths();
        virtual ~IMU_maths();

        void accelerationIntegration1Axe(const DATA_TYPE acceleration, DATA_TYPE& velocity, DATA_TYPE& position, const DATA_TYPE initial_position, const DATA_TYPE initial_velocity, const DATA_TYPE deltaTime);
        void accelerationIntegration3Axes(const Vec3 acceleration, Vec3& velocity, Vec3& position, const Vec3 initial_position, const Vec3 initial_velocity, const DATA_TYPE deltaTime);
        void accelerationIntegration3Axes(Sample& new_sample, Sample& previous_sample);

        void QuaternionToEuler(const Vec4 q, Vec3& v); // convert a quaternion to a euler vector
        void EulerToQuaternion(const Vec3 v, Vec4& q); // convert euler vector to a quaternion
		void QuaternionToOrthogonalMatrix(const Vec4 q, RotMat matrix); // convert quaternion to an orthogonal rotational matrix
		void OrthogonalMatrixToQuaternion(const  RotMat matrix, Vec4& q);  // convert an orthogonal rotational matrix to a quaternion

        void NormalizeQuaternion(Vec4& q); // normalize the matrix (to have |q| = 1)
        DATA_TYPE NormQuaternion(const Vec4 q); // return the norm of the quaternion q

		/* TO DO 
		 * change these two functions to write data coming from several sensors
		 */
        void writeDataInFile(FILE* file, DATA_FORMAT _df, Sample& data, unsigned char data_to_write); // write the data from the sample "data" into the file "file" according to the selected data to write in "data_to_write"
        void writeHeaderInFile(FILE* _file, DATA_FORMAT _df, unsigned char data_to_write);// write the data from the sample "data" into the file "file" according to the selected data to write in "data_to_write"

		void displayMatrix(const RotMat matrix); // display the given matrix
		void addMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result); // result = mat1 + mat2
		void subMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result); // result = mat1 - mat2
		void mulMatrix(const RotMat matrix1, const RotMat matrix2, RotMat result); // result = mat1 * mat2
		void mulMatrix(const RotMat matrix1, const DATA_TYPE real, RotMat result); //result = real * matrix1
		void traMatrix(const RotMat matrix, RotMat result); // result = transpose(matrix)
		void copMatrix(const RotMat matrix1, RotMat matrix2); // mat2 = mat1

    protected:
    private:
        

};
#endif // IMU_MATHS_H
