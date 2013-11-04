#ifndef _MESH_BONED_H_
#define _MESH_BONED_H_

#include "Mesh.h"

class MeshBoned : public Mesh
{
	public:
		struct BonedVertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
			unsigned int boneCount;
			BoneIndex boneIndices;
			BoneWeight boneWeights;
		};

		MeshBoned( int faceCount, int vertexCount, BonedVertexData* vertexData, unsigned int* indices );
		virtual ~MeshBoned();

	protected:
		virtual unsigned int getVertexSize() { return sizeof( MeshBoned::BonedVertexData ); };
		virtual void enableAttributes();
		virtual void disableAttributes();

};

#endif /*GEOM_ANIMATED_MESH_H_*/
