#include "GeomMesh.h"

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
		glDeleteBuffers( 1, &this->m_dataVBO );

	if ( this->m_indexVBO )
		glDeleteBuffers( 1, &this->m_indexVBO );

	if ( this->m_vertexData )
		delete[] this->m_vertexData;

	if ( this->m_indexBuffer )
		delete[] this->m_indexBuffer;
}

bool
GeomMesh::render()
{
	GLint status;

	//GeomType::render();
	
	// lazy loading
	if ( 0 == this->m_dataVBO )
	{
		// generate and setup data vbo
		glGenBuffers( 1, &this->m_dataVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glGenBuffers failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBufferData( GL_ARRAY_BUFFER, sizeof( VertexData ) * this->vertexCount, this->m_vertexData, GL_STATIC_DRAW );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBufferData GL_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer(0) GL_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		// generate and setup index vbo
		glGenBuffers( 1, &this->m_indexVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glGenBuffers indices failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->faceCount * 3, this->m_indexBuffer, GL_STATIC_DRAW );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBufferData GL_ELEMENT_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer(0) GL_ELEMENT_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}
	}

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glEnableVertexAttribArray( 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glEnableVertexAttribArray( 0 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glEnableVertexAttribArray( 1 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glEnableVertexAttribArray( 1 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glEnableVertexAttribArray( 2 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glEnableVertexAttribArray( 2 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 0 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glVertexAttribPointer( 0 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 12 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glVertexAttribPointer( 1 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 24 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glVertexAttribPointer( 2 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER - render - failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glDrawElements( GL_TRIANGLES, this->faceCount * 3, GL_UNSIGNED_INT, BUFFER_OFFSET( 0 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDrawElements failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glDisableVertexAttribArray( 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDisableVertexAttribArray( 0 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}
	
	glDisableVertexAttribArray( 1 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDisableVertexAttribArray( 1 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glDisableVertexAttribArray( 2 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDisableVertexAttribArray( 2 ) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}
