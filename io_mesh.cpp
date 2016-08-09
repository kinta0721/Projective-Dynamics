#include "io_mesh.h"

MeshLoader::MeshLoader()
{
    m_vertices.clear();
    m_faces.clear();
    m_tets.clear();
    load_success = false;
}

// courtesy to Yusuf 
MeshLoader::MeshLoader(char* filename, float scale, glm::vec3 translate)
{
    m_vertices.clear();
    m_faces.clear();
    m_tets.clear();
    load_success = false;

    std::cout << "MeshLoader initializing (to " << filename << ").." << std::endl;

    std::ifstream infile(filename);
    if(!infile.is_open())
    {
        std::cout << "cannot read " << filename << std::endl;
        return;
    }

    char buffer[256];
    glm::vec3 pos;
    Face face; 
    Tet tet; 
    char ignore[256];
    while(!infile.eof())
    {
        infile >> buffer;
        if (strcmp(buffer, "Vertices") == 0)
        {
            int num_vertices;
            infile >> num_vertices;
            for (int i = 0; i < num_vertices; i++)
            {
                infile >> pos.x >> pos.y >> pos.z >> ignore;
                m_vertices.push_back(pos * scale + translate);
            }
            if (m_vertices.size() != num_vertices)
            {
                std::cout << "Init MeshLoader: error on reading vertices." << std::endl;
                return;
            }
        }
        else if (strcmp(buffer, "Triangles") == 0)
        {
            int num_face;
            infile >> num_face;
            for (int i = 0; i < num_face; i++)
            {
                infile >> face.id1 >> face.id2 >> face.id3 >> ignore;
                face.IDMinusMinus();
                m_faces.push_back(face);
            }
            if (m_faces.size() != num_face)
            {
                std::cout << "Init MeshLoader: error on reading faces." << std::endl;
                return;
            }
        }
        else if (strcmp(buffer, "Tetrahedra") == 0)
        {
            int num_tet;
            infile >> num_tet;
            for (int i = 0; i < num_tet; i++)
            {
                infile >> tet.id1 >> tet.id2 >> tet.id3 >> tet.id4 >> ignore;
                tet.IDMinusMinus();
                m_tets.push_back(tet);
            }
            if (m_tets.size() != num_tet)
            {
                std::cout << "Init MeshLoader: error on reading tets." << std::endl;
                return;
            }
        }
    }

    load_success = true;
}

MeshLoader::~MeshLoader()
{
    m_vertices.clear();
    m_faces.clear();
    m_tets.clear();
}
