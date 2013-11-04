#include "MeshBoned.h"

#include "../Util/GLUtils.h"

#include <GL/glew.h>

MeshBoned::MeshBoned( int faceCount, int vertexCount, BonedVertexData* vertexData, unsigned int* indices )
	: Mesh( faceCount, vertexCount, vertexData, indices )
{
}

MeshBoned::~MeshBoned()
{
}

void
MeshBoned::enableAttributes()
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

	// bone-count
	glEnableVertexAttribArray( 4 );
	GL_PEEK_ERRORS_AT_DEBUG

	// bone-indices 
	glEnableVertexAttribArray( 5 );
	GL_PEEK_ERRORS_AT_DEBUG

	// bone-weights
	glEnableVertexAttribArray( 6 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to coordinates
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to normals
	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to texture coords
	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 24 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to tangent
	glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 32 ) );
	GL_PEEK_ERRORS_AT_DEBUG
	
	// specify pointer to bone-count
	glVertexAttribIPointer( 4, 1, GL_UNSIGNED_INT, sizeof( BonedVertexData ), BUFFER_OFFSET( 44 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to bone-indices 
	glVertexAttribIPointer( 5, 4, GL_UNSIGNED_INT, sizeof( BonedVertexData ), BUFFER_OFFSET( 48 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to bone-weights
	glVertexAttribPointer( 6, 4, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 64 ) );
	GL_PEEK_ERRORS_AT_DEBUG
}

void
MeshBoned::disableAttributes()
{
	glDisableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG
	
	glDisableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 3 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 4 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 5 );
	GL_PEEK_ERRORS_AT_DEBUG
					
	glDisableVertexAttribArray( 6 );
	GL_PEEK_ERRORS_AT_DEBUG
}
