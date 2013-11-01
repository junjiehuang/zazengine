#ifndef GEOM_ANIMATED_MESH_H_
#define GEOM_ANIMATED_MESH_H_

#include <GL/glew.h>

#include "GeomType.h"

class GeomAnimatedMesh : public GeomType
{
	public:
		typedef float Vertex [ 3 ];
		typedef float Normal [ 3 ];
		typedef float TexCoord [ 2 ];
		typedef float Tangent [ 3 ];
		typedef unsigned int BoneIndex[ 4 ];
		typedef float BoneWeight[ 4 ];

		struct VertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
			unsigned int boneCount;
			BoneIndex boneIndices;
			BoneWeight boneWeights;
		};

		GeomAnimatedMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices );
		virtual ~GeomAnimatedMesh();

		virtual int getFaceCount() { return this->faceCount; };

		virtual bool render();
	
	private:
		int faceCount;
		int vertexCount;

		VertexData* m_vertexData;
		GLuint* m_indexBuffer;

		GLuint m_dataVBO;
		GLuint m_indexVBO;

};

#endif /*GEOM_ANIMATED_MESH_H_*/
