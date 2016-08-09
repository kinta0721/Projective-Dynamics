#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include <vector>
#include <iostream>

#include "global_headers.h"
#include "math_headers.h"
#include "opengl_headers.h"
#include "primitive.h"

class Constraint
{
public:
    Constraint(ScalarType *stiffness);
    Constraint(const Constraint& other);
    virtual ~Constraint();

    virtual void computeLaplacianMat(std::vector<SparseMatrixTriplet>& l_triplets){std::cout << "Warning: reach <Constraint> base class virtual function." << std::endl; }
    virtual void computeJVector(const EigenMatrixXs& X, EigenMatrixXs& b) { std::cout << "Warning: reach <Constraint> base class virtual function." << std::endl; }

    inline const ScalarType& Stiffness() {return (*m_stiffness);}

protected:
    ScalarType *m_stiffness;

public:
    virtual void draw(const VBO& vbos) { /*do nothing*/ }
};

class AttachmentConstraint : public Constraint
{
public:
    AttachmentConstraint(ScalarType *stiffness);
    AttachmentConstraint(ScalarType *stiffness, uint index, const EigenVector3& fixedpoint);
    AttachmentConstraint(const AttachmentConstraint& other);
    virtual ~AttachmentConstraint();

	virtual void computeLaplacianMat(std::vector<SparseMatrixTriplet>& l_triplets);
	virtual void computeJVector(const EigenMatrixXs& X, EigenMatrixXs& b);

protected:
    uint m_index;
    EigenVector3 m_fixd_point;

// for visualization and selection
public:
    virtual void draw(const VBO& vbos);
    inline  void select()   {m_selected = true;}
    inline  void unSelect() {m_selected = false;}
    inline  EigenVector3 getFixedPoint() {return m_fixd_point;}
    inline  void setFixedPoint(const EigenVector3& target) {m_fixd_point = target;}
    inline  uint getConstrainedVertexIndex() {return m_index;}

private: 
    bool   m_selected;
    Sphere m_attachment_constraint_body;
};

#endif