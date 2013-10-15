#include "Tools.h"

/** For debugging */
void Tools::coutMatrix(const Matrix4x4& matrix)
{
    std::cout<<"START MATRIX"<<std::endl;
    std::cout<<matrix(0, 0)<<" et "<<matrix(0, 1)<<" et "<<matrix(0, 2)<<" et "<< matrix(0, 3)<<std::endl;
    std::cout<<matrix(1, 0)<<" et "<<matrix(1, 1)<<" et "<<matrix(1, 2)<<" et "<< matrix(1, 3)<<std::endl;
    std::cout<<matrix(2, 0)<<" et "<<matrix(2, 1)<<" et "<<matrix(2, 2)<<" et "<< matrix(2, 3)<<std::endl;
    std::cout<<matrix(3, 0)<<" et "<<matrix(3, 1)<<" et "<<matrix(3, 2)<<" et "<< matrix(3, 3)<<std::endl;
    std::cout<<"END MATRIX"<<std::endl;
}

/** For debugging */
void Tools::coutVertex(const Vector3& vertex)
{
    std::cout<<"START VERTEX "<<vertex.x()<<" et "<<vertex.y()<<" et "<<vertex.z()<<" END VERTEX"<<std::endl;
}


Matrix4x4 Tools::translationMatrix(const Vector3& translation) {
	Matrix4x4 mat = Matrix4x4::Identity();
	mat(0,3) = translation[0];
	mat(1,3) = translation[1];
	mat(2,3) = translation[2];
	return mat;
}

Matrix4x4 Tools::scaleMatrix(float scale) {
	return scale*Matrix4x4::Identity();
}

Matrix4x4 Tools::rotationMatrix(const Vector3& axis, float angle) {
	Matrix4x4 mat;
	const float c = std::cosf(angle);
	const float s = std::sinf(angle);
	const float C = (1-c);
	mat << axis[0]*axis[0]*C + c, axis[0]*axis[1]*C - axis[2]*s, axis[0]*axis[2]*C + axis[1]*s, 0,
		axis[1]*axis[0]*C + axis[2]*s, axis[1]*axis[1]*C + c, axis[1]*axis[2]*C - axis[0]*s, 0,
		axis[2]*axis[0]*C - axis[1]*s, axis[2]*axis[1]*C + axis[0]*s, axis[2]*axis[2]*C + c, 0,
		0, 0, 0, 1;
	return mat;
}
