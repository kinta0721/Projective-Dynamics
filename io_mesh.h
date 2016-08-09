#ifndef _IO_MESH_H_
#define _IO_MESH_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "glm.hpp"

class MeshLoader{
public:
    
    struct Face{
        unsigned int id1,id2,id3;
        Face() {}
        Face(int a, int b, int c) : id1(a), id2(b), id3(c){}
        void IDMinusMinus() {id1--; id2--; id3--;}
    };

    struct Tet{
        unsigned int id1,id2,id3,id4;
        Tet() {}
        Tet(int a, int b, int c, int d) : id1(a), id2(b), id3(c), id4(d){}
        void IDMinusMinus() {id1--; id2--; id3--; id4--;}
    };

    MeshLoader();
    MeshLoader(char* filename, float scale = 10.0f, glm::vec3 translate = glm::vec3(0.0f, 4.0f, 0.0f));
    virtual ~MeshLoader();

    inline bool Info() {return load_success;}

    //Vertices, edges, and faces information
    std::vector<glm::vec3> m_vertices;
    std::vector<Face> m_faces;
    std::vector<Tet> m_tets;
    
    bool load_success;
};

#endif