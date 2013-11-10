#ifndef _MESH_STATIC_H_
#define _MESH_STATIC_H_

#include "Mesh.h"

class MeshStatic : public Mesh
{
	public:
		struct StaticVertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
		};

		MeshStatic( int faceCount, int vertexCount, StaticVertexData* vertexData, unsigned int* indices );
		virtual ~MeshStatic();

	protected:
		virtual unsigned int getVertexSize() { return sizeof( MeshStatic::StaticVertexData ); };
		virtual void enableAttributes();
		virtual void disableAttributes();
};

#endif /* _MESH_STATIC_H_ */
