#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>

#include "openGL_headers.h"
#include "math_headers.h"
#include "global_headers.h"
#include "primitive.h"
#include "tinyxml.h"

class Scene;
#ifdef VS2010
class XMLSceneVisitor;	// TinyXML is not available on Visual
#endif // !VS2010

class Scene
{
public:
    Scene(const char* file_name);
    virtual ~Scene();

#ifdef VS2010
    void LoadFromFile(const char* file_name);
#endif // !VS2010
    virtual void Draw(const VBO& vbos);
    void InsertPrimitve(Primitive* const new_primitive);

    bool StaticIntersectionTest(const EigenVector3& p, EigenVector3& normal, ScalarType& dist);

protected:
    std::vector<Primitive*> m_primitives;

private:
};

#ifdef VS2010
class XMLSceneVisitor : public TiXmlVisitor
{
public:
    XMLSceneVisitor(Scene* const scene);
    XMLSceneVisitor(const XMLSceneVisitor& other);
    virtual ~XMLSceneVisitor();

    virtual bool VisitEnter(const TiXmlElement& element, const TiXmlAttribute* attribute);
    virtual bool VisitExit( const TiXmlElement& element);

protected:
    Scene* const m_scene;
    Primitive* m_current;
};
#endif // !VS2010
#endif