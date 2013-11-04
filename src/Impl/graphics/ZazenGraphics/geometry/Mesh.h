#ifndef MESH_H_
#define MESH_H_

#include <glm/glm.hpp>

#define BUFFER_OFFSET( i ) ( ( char* ) NULL + ( i ) )

class Mesh
{
	public:
		typedef float Vertex [ 3 ];
		typedef float Normal [ 3 ];
		typedef float TexCoord [ 2 ];
		typedef float Tangent [ 3 ];
		typedef unsigned int BoneIndex[ 4 ];
		typedef float BoneWeight[ 4 ];

		Mesh( int faceCount, int vertexCount, void* vertexData, unsigned int* indices );
		virtual ~Mesh();

		void compareAndSetBB( const glm::vec3&, const glm::vec3& );
		void setBB( const glm::vec3&, const glm::vec3& );
	 
		const glm::vec3& getCenter() { return this->m_center; };
		const glm::vec3& getBBMin() { return this->m_bbMin; };
		const glm::vec3& getBBMax() { return this->m_bbMax; };

		int getFaceCount() { return this->m_faceCount; };
		int getVertexCount() { return this->m_vertexCount; };

		bool render();

	protected:
		virtual unsigned int getVertexSize() = 0;
		virtual void enableAttributes() = 0;
		virtual void disableAttributes() = 0;

	private:
		void* m_vertexData;
		unsigned int* m_indexBuffer;

		unsigned int m_dataVBO;
		unsigned int m_indexVBO;

		int m_faceCount;
		int m_vertexCount;

		glm::vec3 m_bbMin;
		glm::vec3 m_bbMax;
		glm::vec3 m_center;

};

#endif /*GEOMTYPE_H_*/
