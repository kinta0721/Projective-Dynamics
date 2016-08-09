#pragma warning( disable : 4244 4267)

#include <cassert>

#include "scene.h"

//----------Scene Class----------//
Scene::Scene(const char* file_name)
{
#ifdef VS2010
   LoadFromFile(file_name);
#endif // !VS2010
}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        delete (*iter);
    }
    m_primitives.clear();
}

#ifdef VS2010
void Scene::LoadFromFile(const char* file_name)
{
    TiXmlDocument xml_file(file_name);
    if(xml_file.LoadFile())
    {
        XMLSceneVisitor visitor(this);
        xml_file.Accept(&visitor);
    }
}
#endif // !VS2010

void Scene::Draw(const VBO& vbos)
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        (*iter)->Draw(vbos);
    }
}

bool Scene::StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist)
{
    dist = 0;

    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        ScalarType d;
        EigenVector3 n;
        if((*iter)->StaticIntersectionTest(p, n, d))
        {
            if (d < dist)
            {
                dist = d;
                normal = n;
            }
        }
    }

    if (dist < 0)
        return true;
    else
        return false;
}

void Scene::InsertPrimitve(Primitive* const new_primitive)
{
    m_primitives.push_back(new_primitive);
}

#ifdef VS2010
//----------Scene Visitor Class----------//
XMLSceneVisitor::XMLSceneVisitor(Scene* const scene) : 
    TiXmlVisitor(),
    m_scene(scene),
    m_current(NULL)
{
    ;
}

XMLSceneVisitor::XMLSceneVisitor(const XMLSceneVisitor& other) : 
    TiXmlVisitor(other),
    m_scene(other.m_scene),
    m_current(other.m_current)
{
    ;
}

XMLSceneVisitor::~XMLSceneVisitor()
{
    ;
}

bool XMLSceneVisitor::VisitEnter(const TiXmlElement& element, const TiXmlAttribute* attribute)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return (element.Parent()->ValueStr() == "scene");
    }
    else if (element.ValueStr() == "plane")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double nx(0.0), ny(1.0), nz(0.0), value(0.0);

        element.Attribute("nx", &nx);
        element.Attribute("ny", &ny);
        element.Attribute("nz", &nz);
        element.Attribute("value", &value);

        glm::vec3 normal(nx, ny, nz);
        normal = glm::normalize(normal);

        m_current = new Plane(normal, value);
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0), cy(1.0), cz(0.0), radius(0.0);

        element.Attribute("cx", &cx);
        element.Attribute("cy", &cy);
        element.Attribute("cz", &cz);
        element.Attribute("radius", &radius);

        glm::vec3 center(cx, cy, cz);
        m_current = new Sphere(center, radius);
        return true;
    }
    else if (element.ValueStr() == "cube")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0), cy(1.0), cz(0.0), hx(0.0), hy(0.0), hz(0.0);

        element.Attribute("cx", &cx);
        element.Attribute("cy", &cy);
        element.Attribute("cz", &cz);
        element.Attribute("hx", &hx);
        element.Attribute("hy", &hy);
        element.Attribute("hz", &hz);

        glm::vec3 center(cx, cy, cz);
        glm::vec3 hf_dims(hx, hy, hz);
        m_current = new Cube(center, hf_dims);
        return true;
    }
    else if (element.ValueStr() == "obj")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0), cy(1.0), cz(0.0), scale(0.0);

        element.Attribute("cx", &cx);
        element.Attribute("cy", &cy);
        element.Attribute("cz", &cz);
        element.Attribute("scale", &scale);

        glm::vec3 center(cx, cy, cz);
        m_current = new ObjMesh(DEFAULT_OBJ_MODEL, center, scale);
        return true;
    }
    else
        return false;
}

bool XMLSceneVisitor::VisitExit( const TiXmlElement& element)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return true;
    }
    else if (element.ValueStr() == "plane")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
    else if (element.ValueStr() == "cube")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
    else if (element.ValueStr() == "obj")
    {
        m_scene->InsertPrimitve(m_current);
        m_current = NULL;
        return true;
    }
    else
        return false;
}
#endif // !VS2010