#include "constraint.h"

#pragma region Constraint
Constraint::Constraint(ScalarType *stiffness) : 
    m_stiffness(stiffness)
{
}

Constraint::Constraint(const Constraint& other) : 
    m_stiffness(other.m_stiffness)
{
}

Constraint::~Constraint()
{
}
#pragma endregion

#pragma region Attachment Constraint
AttachmentConstraint::AttachmentConstraint(ScalarType *stiffness) : 
    Constraint(stiffness)
{
    m_selected = false;
}

AttachmentConstraint::AttachmentConstraint(ScalarType *stiffness, uint index, const EigenVector3& fixedpoint) : 
    Constraint(stiffness),
    m_index(index),
    m_fixd_point(fixedpoint)
{
    m_selected = false;
}

AttachmentConstraint::AttachmentConstraint(const AttachmentConstraint& other) : 
    Constraint(other),
    m_index(other.m_index),
    m_fixd_point(other.m_fixd_point),
    m_selected(other.m_selected)
{
    
}

AttachmentConstraint::~AttachmentConstraint()
{
}

void AttachmentConstraint::computeLaplacianMat(std::vector<SparseMatrixTriplet>& l_triplets)
{
	ScalarType ks = *m_stiffness * 0.5;
	// Inprement Here !! //
	// Hint: use "ks"
	
		l_triplets.push_back(SparseMatrixTriplet(m_index , m_index , ks));
	
}

void AttachmentConstraint::computeJVector(const EigenMatrixXs& X, EigenMatrixXs& b)
{
	EigenVector3 v = 0.5 * (*m_stiffness) * m_fixd_point;
	// Inprement Here !! //
	// Hint: use "v"
	for (int i = 0; i < 3; i++) {

		b(m_index, i) = X(m_index, i) + v(i);// -m_ExternalForce.coeff(i, j);

	}
}

void AttachmentConstraint::draw(const VBO& vbos)
{
    m_attachment_constraint_body.move_to(Eigen2GLM(m_fixd_point));
    if (m_selected)
        m_attachment_constraint_body.change_color(glm::vec3(0.8, 0.8, 0.2));
    else
        m_attachment_constraint_body.change_color(glm::vec3(0.8, 0.2, 0.2));
        
    m_attachment_constraint_body.Draw(vbos);
}
#pragma endregion