#include "Mesh.h"

#include "../Util/GLUtils.h"

#include <GL/glew.h>

Mesh::Mesh( GLuint vao, GLuint dataVBO, GLuint indexVBO, int indicesCount )
	: m_vao( vao ), 
	m_dataVBO( dataVBO ),
	m_indexVBO( indexVBO ),
	m_indicesCount( indicesCount )
{
	// will be set by geometry-factory
	// TODO: no need to store the vertex- & index-data as it is hold by OpenGL, can be deleted
	this->m_vertexData = NULL;
	this->m_indexData = NULL;
}

Mesh::~Mesh()
{
	if ( this->m_vao )
	{
		glDeleteVertexArrays( 1, &this->m_vao );
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

	if ( this->m_indexData )
	{
		delete[] this->m_indexData;
	}
}

bool
Mesh::render()
{
	glBindVertexArray( this->m_vao );
	GL_PEEK_ERRORS_AT_DEBUG

	glDrawElements( GL_TRIANGLES, this->m_indicesCount, GL_UNSIGNED_INT, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

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
