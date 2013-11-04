#include "Mesh.h"

#include "../Util/GLUtils.h"

#include <GL/glew.h>

Mesh::Mesh( const std::string& name )
{
	this->m_faceCount = 0;
	this->m_vertexCount = 0;

	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	this->m_vertexData = 0;
	this->m_indexBuffer = 0;
}

Mesh::Mesh( const std::string& name, int faceCount, int vertexCount, void* vertexData, unsigned int* indices )
	: m_name( name ),
	m_faceCount( faceCount ),
	m_vertexCount( vertexCount )
{
	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	this->m_vertexData = vertexData;
	this->m_indexBuffer = indices;
}

Mesh::~Mesh()
{
	for ( unsigned int i = 0; i < this->m_children.size(); i++ )
	{
		delete this->m_children[ i ];
	}

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
Mesh::render()
{
	if ( 0 == this->m_vertexData || 0 == this->m_indexBuffer )
	{
		return true;
	}

	// lazy loading
	if ( 0 == this->m_dataVBO )
	{
		// generate and setup data vbo
		glGenBuffers( 1, &this->m_dataVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBufferData( GL_ARRAY_BUFFER, this->getVertexSize() * this->m_vertexCount, this->m_vertexData, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT_DEBUG

		// generate and setup index vbo
		glGenBuffers( 1, &this->m_indexVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
		GL_PEEK_ERRORS_AT_DEBUG

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->m_faceCount * 3, this->m_indexBuffer, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	this->enableAttributes();

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glDrawElements( GL_TRIANGLES, this->m_faceCount * 3, GL_UNSIGNED_INT, BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	this->disableAttributes();

	return true;
}

void
Mesh::compareAndSetBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	for ( int i = 0; i < 3; i++ )
	{
		if ( bbMax[ i ] > this->m_bbMax[ i ] )
		{
			this->m_bbMax[ i ] = bbMax[ i ];
		}
		else if ( bbMin[ i ] < this->m_bbMin[ i ] )
		{
			this->m_bbMin[ i ] = bbMin[ i ];
		}
	}

	this->setBB( this->m_bbMin, this->m_bbMax );
}
 
void
Mesh::setBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	this->m_bbMin = bbMin;
	this->m_bbMax = bbMax;
	
	for ( int i = 0; i < 3; i++ )
	{
		this->m_center[ i ] = this->m_bbMin[ i ] + ( ( this->m_bbMax[ i ] - this->m_bbMin[ i ] ) / 2 );
	}
}
