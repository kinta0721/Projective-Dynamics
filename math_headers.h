#ifndef _MATH_HEADERS_H_
#define _MATH_HEADERS_H_

#include <iostream>

// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/unsupported/Eigen/SparseExtra>

// glm
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

// global header
#include "global_headers.h"

// eigen vectors and matrices
typedef int IndexType;
typedef unsigned int uint;
typedef unsigned char uchar;

typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2, 1> EigenVector2;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3, 1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 4, 1, 0, 4, 1> EigenVector4;
typedef Eigen::Matrix<ScalarType, 9, 1, 0, 9, 1> EigenVector9;
typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, 1> EigenVectorXi;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> EigenVectorXs;

typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2, 2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3, 3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 4, 4, 0, 4, 4> EigenMatrix4;
typedef Eigen::Matrix<IndexType,  Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXi;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXs;

typedef Eigen::Quaternion<ScalarType> EigenQuaternion;

typedef Eigen::SparseMatrix<ScalarType> SparseMatrix;
typedef Eigen::Triplet<ScalarType,IndexType> SparseMatrixTriplet;

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)
#define block_matrix(a) block<9,1>(9*(a), 0)
#define block_quaternion(a) block<4,1>(4*(a), 0)

// eigen 2 glm, glm 2 eigen
glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector);
EigenVector3 GLM2Eigen(const glm::vec3& glm_vector);

// solver
void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = "");
void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg = "");
#endif