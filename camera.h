#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "openGL_headers.h"
#include "global_headers.h"
#include "math_headers.h"
#include "mesh.h"

//forward declaration
class Mesh;

class Camera
{
public:
    // constructor and destructor
    Camera(void);
    virtual ~Camera(void);

    // reset
    void Reset(int width, int height);
    void Lookat(Mesh* mesh);
    
    // get camera matrices:
    inline glm::mat4 GetViewMatrix() {return m_view;}
    inline glm::mat4 GetProjectionMatrix() {return m_projection;}
    // get camera position and raycast direction:
    inline glm::vec3 GetCameraPosition() {return m_position;}
    inline float GetCameraDistance() {return m_eye_distance;}
    inline void SetProjectionPlaneDistance(float distance) {m_cached_projection_plane_distance = distance;}
    glm::vec3 GetRaycastDirection(int x, int y);
    glm::vec3 GetCurrentTargetPoint(int x, int y);

    // mouse interactions
    void MouseChangeDistance(float coe, float dx, float dy);
    void MouseChangeLookat(float coe, float dx, float dy);
    void MouseChangeHeadPitch(float coe, float dx, float dy);

    // Draw axis
    void DrawAxis();

    // resize
    void ResizeWindow(int w, int h);

protected:
    int m_width;
    int m_height;
    float m_znear;
    float m_zfar;
    float m_fovy;

    float m_eye_distance;
    float m_head;
    float m_pitch;

    glm::vec3 m_position;
    glm::vec3 m_up;
    glm::vec3 m_lookat;
    glm::vec3 m_cached_projection_plane_center;
    glm::vec3 m_cached_projection_plane_xdir;
    glm::vec3 m_cached_projection_plane_ydir;
    float m_cached_projection_plane_distance;

    glm::mat4 m_view;
    glm::mat4 m_projection;
private:
    // update camera matrices:
    void updateViewMatrix();
    void updateProjectionMatrix();
};

#endif