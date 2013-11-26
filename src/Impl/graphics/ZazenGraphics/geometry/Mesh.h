#ifndef _MESH_H_
#define _MESH_H_

#include <glm/glm.hpp>

#include <GL/glew.h>

class Mesh
{
	public:
		friend class GeometryFactory;

		Mesh( GLuint vao, GLuint dataVBO, GLuint indexVBO, int indicesCount );
		~Mesh();
	 
		const glm::vec3& getCenter() { return this->m_center; };
		const glm::vec3& getBBMin() { return this->m_bbMin; };
		const glm::vec3& getBBMax() { return this->m_bbMax; };

		bool render();

	private:
		GLuint m_vao;
		GLuint m_dataVBO;
		GLuint m_indexVBO;
		GLenum m_mode;
		int m_indicesCount;

		void* m_vertexData;
		GLuint* m_indexData;

		glm::vec3 m_bbMin;
		glm::vec3 m_bbMax;
		glm::vec3 m_center;

		void setBB( const glm::vec3&, const glm::vec3& );
		void compareAndSetBB( const glm::vec3&, const glm::vec3& );
};

#endif /* _MESH_H_ */
