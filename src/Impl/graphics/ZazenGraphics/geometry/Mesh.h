#ifndef MESH_H_
#define MESH_H_

#include <glm/glm.hpp>

#include <vector>
#include <string>

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

		Mesh( const std::string& name ); // act as empty parent for composite
		Mesh( const std::string& name, int faceCount, int vertexCount, void* vertexData, unsigned int* indices );
		virtual ~Mesh();

		const std::string& getName() const { return this->m_name; };

		void compareAndSetBB( const glm::vec3&, const glm::vec3& );
		void setBB( const glm::vec3&, const glm::vec3& );
	 
		const glm::vec3& getCenter() { return this->m_center; };
		const glm::vec3& getBBMin() { return this->m_bbMin; };
		const glm::vec3& getBBMax() { return this->m_bbMax; };

		const glm::mat4& getModelMatrix() const { return this->m_modelMatrix; };
		void setModelMatrix( const glm::mat4& mat ) { this->m_modelMatrix = mat; };

		void addChild( Mesh* child ) { this->m_children.push_back( child ); };
		const std::vector<Mesh*>& getChildren() const { return this->m_children; };

		int getFaceCount() { return this->m_faceCount; };
		int getVertexCount() { return this->m_vertexCount; };

		bool render();

	protected:
		virtual unsigned int getVertexSize() { return 0; };
		virtual void enableAttributes() {};
		virtual void disableAttributes() {};

	private:
		std::vector<Mesh*> m_children;
	 
		void* m_vertexData;
		unsigned int* m_indexBuffer;

		unsigned int m_dataVBO;
		unsigned int m_indexVBO;

		int m_faceCount;
		int m_vertexCount;

		std::string m_name;
		glm::mat4 m_modelMatrix;

		glm::vec3 m_bbMin;
		glm::vec3 m_bbMax;
		glm::vec3 m_center;

};

#endif /*GEOMTYPE_H_*/
