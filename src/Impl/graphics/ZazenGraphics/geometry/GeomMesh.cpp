#include "GeomMesh.h"

#include "../ZazenGraphics.h"
#include "../Util/GLUtils.h"

#include <iostream>

using namespace std;

#define BUFFER_OFFSET( i ) ( ( char* ) NULL + ( i ) )

GeomMesh::GeomMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices )
	: faceCount( faceCount ),
	  vertexCount( vertexCount )
{
	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	this->m_vertexData = data;
	this->m_indexBuffer = indices;
}

GeomMesh::~GeomMesh()
{
	if ( this->m_dataVBO )
	{
		glDeleteBuffers( 1, &this->m_dataVBO );
	}

	if ( this->m_indexVBO )
	{
		glDeleteBuffers( 1, &this->m_indexVBO );
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
GeomMesh::render()
{
	// lazy loading
	if ( 0 == this->m_dataVBO )
	{
		// generate and setup data vbo
		glGenBuffers( 1, &this->m_dataVBO );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glGenBuffers failed";
			return false;
		}

		glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed";
			return false;
		}

		glBufferData( GL_ARRAY_BUFFER, sizeof( VertexData ) * this->vertexCount, this->m_vertexData, GL_STATIC_DRAW );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBufferData GL_ARRAY_BUFFER failed";
			return false;
		}

		// generate and setup index vbo
		glGenBuffers( 1, &this->m_indexVBO );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glGenBuffers indices failed";
			return false;
		}

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER failed";
			return false;
		}

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->faceCount * 3, this->m_indexBuffer, GL_STATIC_DRAW );
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBufferData GL_ELEMENT_ARRAY_BUFFER failed";
			return false;
		}
	}

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 24 ) );
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

	return true;
}
