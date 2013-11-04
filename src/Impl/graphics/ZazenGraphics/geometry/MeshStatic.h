#ifndef _STATIC_MESH_H_
#define _STATIC_MESH_H_

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

		MeshStatic( const std::string& name, int faceCount, int vertexCount, StaticVertexData* vertexData, unsigned int* indices );
		virtual ~MeshStatic();

	protected:
		virtual unsigned int getVertexSize() { return sizeof( MeshStatic::StaticVertexData ); };
		virtual void enableAttributes();
		virtual void disableAttributes();

};

#endif /*GEOM_STATIC_MESH_H_*/
