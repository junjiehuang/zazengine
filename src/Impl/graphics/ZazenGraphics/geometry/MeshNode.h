#ifndef _MESHNODE_H_
#define _MESHNODE_H_

#include "Mesh.h"

#include <vector>
#include <string>

class MeshNode
{
	public:
		friend class GeometryFactory;

		MeshNode( const std::string& name ); // act as empty parent for composite
		~MeshNode();

		const std::string& getName() const { return this->m_name; };
		
		void compareAndSetBB( const glm::vec3&, const glm::vec3& );
		void setBB( const glm::vec3&, const glm::vec3& );
	 
		bool hasMeshes() const { return this->m_hasMeshesFlag; };

		const glm::vec3& getCenter() { return this->m_center; };
		const glm::vec3& getBBMin() { return this->m_bbMin; };
		const glm::vec3& getBBMax() { return this->m_bbMax; };

		const glm::mat4& getLocalTransform() const { return this->m_localTransform; };
		const glm::mat4& getGlobalTransform() const { return this->m_globalTransform; };

		const std::vector<MeshNode*>& getChildren() const { return this->m_children; };
		const std::vector<Mesh*>& getMeshes() const { return this->m_meshes; };
		const glm::mat4* getBoneOffset() const { return this->m_boneOffset; };

	private:
		// is true if this node or any child has a non-zero vector of meshes - is calculated during loading
		bool m_hasMeshesFlag;

		std::vector<MeshNode*> m_children;
		std::vector<Mesh*> m_meshes;
		glm::mat4* m_boneOffset; // is NULL if this node is not associated with a bone

		std::string m_name;
		glm::mat4 m_localTransform;
		glm::mat4 m_globalTransform; // accumulated local-transforms from parents

		glm::vec3 m_bbMin;
		glm::vec3 m_bbMax;
		glm::vec3 m_center;
};

#endif /* _MESHNODE_H_ */
