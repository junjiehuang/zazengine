#ifndef GEOMTYPE_H_
#define GEOMTYPE_H_

#include <glm/glm.hpp>

#include <vector>
#include <string>

class GeomType
{
	public:
		 GeomType();
		 virtual ~GeomType();

		 void compareBB( const glm::vec3&, const glm::vec3& );
	 
		 const glm::vec3& getCenter() { return this->m_center; };
		 const glm::vec3& getBBMin() { return this->m_bbMin; };
		 const glm::vec3& getBBMax() { return this->m_bbMax; };
	 
		 void setBB( const glm::vec3&, const glm::vec3& );

		 virtual int getFaceCount() { return 0; };
	 
		 virtual bool render() { return true; };

		 const glm::mat4& getModelMatrix() const { return this->m_modelMatrix; };
		 void setModelMatrix( const glm::mat4& mat ) { this->m_modelMatrix = mat; };

		 void addChild( GeomType* child ) { this->m_children.push_back( child ); };
		 const std::vector<GeomType*>& getChildren() const { return this->m_children; };

		 void setName( const std::string& name ) { this->m_name = name; };

	private:
		 std::vector<GeomType*> m_children;
	 
		 glm::mat4 m_modelMatrix;
		 std::string m_name;

		 glm::vec3 m_bbMin;
		 glm::vec3 m_bbMax;
		 glm::vec3 m_center;

};

#endif /*GEOMTYPE_H_*/
