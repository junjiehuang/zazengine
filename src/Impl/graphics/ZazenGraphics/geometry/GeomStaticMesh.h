#ifndef GEOM_STATIC_MESH_H_
#define GEOM_STATIC_MESH_H_

#include <GL/glew.h>

#include "GeomType.h"

class GeomStaticMesh : public GeomType
{
	public:
		typedef float Vertex [ 3 ];
		typedef float Normal [ 3 ];
		typedef float TexCoord [ 2 ];
		typedef float Tangent [ 3 ];

		struct VertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
		};

		GeomStaticMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices );
		virtual ~GeomStaticMesh();

		virtual int getFaceCount() { return this->faceCount; };

		bool render();
	
	 private:
		int faceCount;
		int vertexCount;

		VertexData* m_vertexData;
		GLuint* m_indexBuffer;

		GLuint m_dataVBO;
		GLuint m_indexVBO;

};

#endif /*GEOM_STATIC_MESH_H_*/
