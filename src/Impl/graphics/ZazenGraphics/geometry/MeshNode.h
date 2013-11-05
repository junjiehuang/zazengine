#ifndef MESHNODE_H_
#define MESHNODE_H_

#include "Mesh.h"

#include <vector>
#include <string>

class MeshNode
{
	public:
		friend class GeometryFactory;

		struct MeshBone {
			std::string m_name;
			glm::mat4 m_offset; // mesh to bone
		};

		MeshNode( const std::string& name ); // act as empty parent for composite
		~MeshNode();

		const std::string& getName() const { return this->m_name; };
		
		void compareAndSetBB( const glm::vec3&, const glm::vec3& );
		void setBB( const glm::vec3&, const glm::vec3& );
	 
		const glm::vec3& getCenter() { return this->m_center; };
		const glm::vec3& getBBMin() { return this->m_bbMin; };
		const glm::vec3& getBBMax() { return this->m_bbMax; };

		const glm::mat4& getModelMatrix() const { return this->m_modelMatrix; };
		const std::vector<MeshNode*>& getChildren() const { return this->m_children; };
		const std::vector<Mesh*>& getMeshes() const { return this->m_meshes; };
		const MeshBone* getBone() const { return this->m_bone; };

	private:
		std::vector<MeshNode*> m_children;
		std::vector<Mesh*> m_meshes;
		MeshBone* m_bone;

		std::string m_name;
		glm::mat4 m_modelMatrix;

		glm::vec3 m_bbMin;
		glm::vec3 m_bbMax;
		glm::vec3 m_center;

};

#endif /*GEOMTYPE_H_*/
