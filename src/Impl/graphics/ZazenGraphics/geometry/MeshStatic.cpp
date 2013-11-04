#include "MeshStatic.h"

#include "../Util/GLUtils.h"

#include <GL/glew.h>

MeshStatic::MeshStatic( int faceCount, int vertexCount, StaticVertexData* vertexData, unsigned int* indices )
	: Mesh( faceCount, vertexCount, vertexData, indices )
{
}

MeshStatic::~MeshStatic()
{
}

void
MeshStatic::enableAttributes()
{
	// coordinates
	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// normals
	glEnableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	// texture-coords
	glEnableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	// tangent
	glEnableVertexAttribArray( 3 );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 24 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 32 ) );
	GL_PEEK_ERRORS_AT_DEBUG

}

void
MeshStatic::disableAttributes()
{
	glDisableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG
	
	glDisableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 3 );
	GL_PEEK_ERRORS_AT_DEBUG
}
