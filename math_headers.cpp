#include "math_headers.h"

glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector)
{
    return glm::vec3(eigen_vector[0], eigen_vector[1], eigen_vector[2]);
}

EigenVector3 GLM2Eigen(const glm::vec3& glm_vector)
{
    return EigenVector3(glm_vector[0], glm_vector[1], glm_vector[2]);
}

void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	SparseMatrix I(A.cols(), A.rows());
	I.setIdentity();
	

	lltSolver.analyzePattern(A_prime);

	lltSolver.factorize(A_prime);
	ScalarType Regularization = 0.00001;
	bool success = true;
	while (lltSolver.info() != Eigen::Success)
	{
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		lltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(llt solver)" << std::endl;
}

void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg)
{
	SparseMatrix A_prime = A;
	SparseMatrix I(A.cols(), A.rows());
	I.setIdentity();

	ldltSolver.analyzePattern(A_prime);
	ldltSolver.factorize(A_prime);
	ScalarType Regularization = 0.00001;
	bool success = true;
	while (ldltSolver.info() != Eigen::Success)
	{
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		ldltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(ldlt solver)" << std::endl;
}