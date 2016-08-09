#ifndef _OPENGL_HEADERS_H_
#define _OPENGL_HEADERS_H_

// opengl headers
#include "GL/glew.h"
#include "GL/freeglut.h" // note: glut needs to be included after glew

struct VBO
{
    VBO()
    {
        if(!glIsBuffer(m_vbo))
            glGenBuffers(1, &m_vbo);
        if(!glIsBuffer(m_cbo))
            glGenBuffers(1, &m_cbo);
        if(!glIsBuffer(m_nbo))
            glGenBuffers(1, &m_nbo);
        if(!glIsBuffer(m_tbo))
            glGenBuffers(1, &m_tbo);
        if(!glIsBuffer(m_ibo))
            glGenBuffers(1, &m_ibo);
    }

    virtual ~VBO()
    {
        if(glIsBuffer(m_vbo))
            glDeleteBuffers(1, &m_vbo);
        if(glIsBuffer(m_cbo))
            glDeleteBuffers(1, &m_cbo);
        if(glIsBuffer(m_nbo))
            glDeleteBuffers(1, &m_nbo);
        if(glIsBuffer(m_tbo))
            glDeleteBuffers(1, &m_tbo);
        if(glIsBuffer(m_ibo))
            glDeleteBuffers(1, &m_ibo);
    }
    // vertex, color, normal, texture, index
    GLuint m_vbo, m_cbo, m_nbo, m_tbo, m_ibo;

    //0: modelview; 1: projection; 2: transformation; 3: enable_texture; 4: texture_sampler
    GLuint m_uniform_modelview;
    GLuint m_uniform_projection;
    GLuint m_uniform_transformation;
    GLuint m_uniform_enable_texture;
    GLuint m_uniform_texture_sampler;
};

#endif