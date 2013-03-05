#ifndef GEOMTYPE_H_
#define GEOMTYPE_H_

#include <GL/glew.h>

#include <glm/glm.hpp>

#include <vector>
#include <string>

class GeomType
{
 public:
	 GeomType();
	 virtual ~GeomType();

	 void compareBB(const glm::vec3&, const glm::vec3&);
	 
	 const glm::vec3& getCenter() { return this->center; };
	 const glm::vec3& getBBMin() { return this->bbMin; };
	 const glm::vec3& getBBMax() { return this->bbMax; };
	 
	 void setBB(const glm::vec3&, const glm::vec3&);
	 
	 virtual int getFaceCount() { return 0; };
	 
	 void renderBB();
	 virtual bool render();

	 GeomType* parent;
	 std::vector<GeomType*> children;
	 
	 glm::mat4 m_modelMatrix;
	 std::string name;

 private:
	 GLuint bbVBO;
		
	 glm::vec3 bbMin;
	 glm::vec3 bbMax;
	 glm::vec3 center;

};

#endif /*GEOMNODE_H_*/
