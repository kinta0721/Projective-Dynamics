#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include <vector>

#include "global_headers.h"
#include "anttweakbar_wrapper.h"
#include "mesh.h"
#include "constraint.h"
#include "scene.h"


class Mesh;
class AntTweakBarWrapper;

// integration method
typedef enum
{
    INTEGRATION_LOCAL_GLOBAL,
    INTEGRATION_TOTAL_NUM

} IntegrationMethod;

// material type
typedef enum
{
	MATERIAL_COROTATIONAL_LINEAR,
	MATERIAL_TOTAL_NUM
}MaterialType;

class Simulation
{
    friend class AntTweakBarWrapper;

public:
    Simulation();
    virtual ~Simulation();

    void reset();
    void update();
    void drawConstraints(const VBO& vbos);

    // select/unselect/move attachment constratins
    ScalarType tryToSelectAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir); // return ray_projection_plane_distance if hit; return -1 otherwise.
    bool tryToToggleAttachmentConstraint(const EigenVector3& p0, const EigenVector3& dir);       // true if hit some vertex/constraint
    void selectAtttachmentConstraint(AttachmentConstraint* ac);
    void unselectAttachmentConstraint();
    void addAttachmentConstraint(uint vertex_index); // add one attachment constraint at vertex_index
    void moveSelectedAttachmentConstraintTo(const EigenVector3& target); // move selected attachement constraint to target

    // inline functions
	inline void setReprecomputeFlag() { m_precomputation_flag = false; }
    inline void setReprefactorFlag()  { m_prefactorization_flag = false; }
    inline void setMesh(Mesh* mesh)   { m_mesh = mesh; }
    inline void setScene(Scene* scene){ m_scene = scene; }
    
protected:
	// flags
	bool m_precomputation_flag;
	bool m_prefactorization_flag;

	// variables
	EigenMatrixXs* m_V;
	EigenMatrixXi* m_F;
	EigenMatrixXi* m_T;
	EigenMatrixXs  m_Vel;

	SparseMatrix m_MassMat;
	SparseMatrix m_Inertia;
	EigenMatrixXs m_ExternalForce;

    // simulation constants
    ScalarType   m_h; // time_step
	uint m_iterations_per_frame; // for optimization method, number of iterations

	// material type and integration method
	IntegrationMethod m_integration_method;
	MaterialType      m_material_type;

    // simulation constants
    ScalarType m_gravity_constant;
    ScalarType m_damping_coefficient;

	ScalarType m_young,   m_young_old;
	ScalarType m_poisson, m_poisson_old;
	ScalarType m_myu;
	ScalarType m_lambda;

	ScalarType m_stiffness_attachment;

    // mesh and scene
    Mesh  *m_mesh;
    Scene *m_scene;

    // constraints
    std::vector<Constraint*> m_constraints;
    AttachmentConstraint*    m_selected_attachment_constraint;

	// for precomputation
	std::vector<EigenMatrix3> m_B; // Dm inverses
	std::vector<ScalarType>   m_W; // volume of Tets

    // for prefactorization
    SparseMatrix m_LaplacianMat;
    SparseMatrix m_JacobianMat;
    Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_LLTsolver;

private:

    // main update sub-routines
	void preComputation();

    void clearConstraints(); // cleanup all constraints
    void setupConstraints(); // initialize constraints

	void convertLameConstant();

    void dampVelocity(); // damp velocity at the end of each iteration.

    void computeInertia(); // calculate the inertia term: y = current_pos + current_vel*h
    void computeExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane

    void collisionDetection(EigenMatrixXs& x, EigenMatrixXs& v); // detect collision and return a vector of penetration

	void singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A);

	// integration scheme
    void integrateOptimizationMethod();
    bool integrateLocalGlobalOneIteration(EigenMatrixXs& x);

    // for local global method 
	inline void computeRotMat(EigenMatrixXs& RotMat, const EigenMatrixXs& Jv);
	void computeElementLaplacianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], std::vector<SparseMatrixTriplet>& l_triplets);
    void computeElementJacobianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], const unsigned int ele_num, std::vector<SparseMatrixTriplet>& j_triplets);
	void volumeconservation(EigenMatrix3 &F, EigenMatrix3 &B, const unsigned int tet_list[], EigenMatrixXs &X);
    // matrices and prefactorizations
    void setLaplacianMat();
    void setJacobianMat();
    void prefactorize();
	double clamp(double n, double lower, double upper);

    // utility functions
    void updatePosAndVel(const EigenMatrixXs NewPos);
	};
#endif