#include "GeomAnimatedMesh.h"

#include "../ZazenGraphics.h"
#include "../Util/GLUtils.h"

#include <iostream>

using namespace std;

#define BUFFER_OFFSET( i ) ( ( char* ) NULL + ( i ) )

GeomAnimatedMesh::GeomAnimatedMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices )
	: faceCount( faceCount ),
	  vertexCount( vertexCount )
{
	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	this->m_vertexData = data;
	this->m_indexBuffer = indices;
}

GeomAnimatedMesh::~GeomAnimatedMesh()
{
	if ( this->m_dataVBO )
	{
		glDeleteBuffers( 1, &this->m_dataVBO );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	if ( this->m_indexVBO )
	{
		glDeleteBuffers( 1, &this->m_indexVBO );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	if ( this->m_vertexData )
	{
		delete[] this->m_vertexData;
	}

	if ( this->m_indexBuffer )
	{
		delete[] this->m_indexBuffer;
	}
}

bool
GeomAnimatedMesh::render()
{
	// lazy loading
	if ( 0 == this->m_dataVBO )
	{
		// generate and setup data vbo
		glGenBuffers( 1, &this->m_dataVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBufferData( GL_ARRAY_BUFFER, sizeof( VertexData ) * this->vertexCount, this->m_vertexData, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT_DEBUG

		// generate and setup index vbo
		glGenBuffers( 1, &this->m_indexVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->faceCount * 3, this->m_indexBuffer, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

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

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 24 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 32 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribIPointer( 4, 1, GL_UNSIGNED_INT, sizeof( VertexData ), BUFFER_OFFSET( 44 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribIPointer( 5, MAX_BONES_PER_VERTEX, GL_UNSIGNED_INT, sizeof( VertexData ), BUFFER_OFFSET( 48 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 6, MAX_BONES_PER_VERTEX, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 48 + sizeof( BoneIndex ) ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glDrawElements( GL_TRIANGLES, this->faceCount * 3, GL_UNSIGNED_INT, BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

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

	return true;
}
