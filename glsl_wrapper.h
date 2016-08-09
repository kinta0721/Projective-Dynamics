#ifndef _GLSL_WRAPPER_H_
#define _GLSL_WRAPPER_H_

#include "global_headers.h"
#include "openGL_headers.h"
#include "camera.h"
#include "stb_image.h"

class RenderWrapper
{
public:
    RenderWrapper(void);
    virtual ~RenderWrapper(void);

    void InitShader(const char* vert_path, const char* frag_path);
    bool InitTexture(const char* tex_path);
    void CleanupShader();

    void SetCameraProjection(glm::mat4 projection);
    void SetCameraModelview(glm::mat4 modelview);

    void ActivateShaderprog();
    void DeactivateShaderprog();
    
public: // inlines
    inline VBO& getVBO() {return m_vbo_handle;}

private: 
    VBO m_vbo_handle;
    GLuint m_vert_handle, m_frag_handle, m_shaderprog_handle;
    GLuint m_texture;

private: // private methods

    //helper function to read shader source and put it in a char array
    //thanks to Swiftless
    char* textFileRead(const char*);

    //some other helper functions from CIS 565
    void printLinkInfoLog(int);
    void printShaderInfoLog(int);
};

#endif