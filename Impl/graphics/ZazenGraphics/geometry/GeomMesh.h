#ifndef MESHNODE_H_
#define MESHNODE_H_

#include <GL/glew.h>

#include "GeomType.h"

class GeomMesh : public GeomType
{
 public:
	typedef float Vertex [ 3 ];
	typedef float Normal [ 3 ];

	struct VertexData
	{
		Vertex position;
		Normal normal;
	};

	GeomMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices );
	virtual ~GeomMesh();

	virtual int getFaceCount() { return this->faceCount; };

	bool render();
	
 private:
	int faceCount;
	int vertexCount;

	VertexData* data;
	GLuint* indexBuffer;

	GLuint dataVBO;
	GLuint indexVBO;

};

#endif /*MESHNODE_H_*/
