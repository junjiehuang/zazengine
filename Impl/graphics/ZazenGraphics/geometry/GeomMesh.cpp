#include "GeomMesh.h"

#include <iostream>

using namespace std;

#define BUFFER_OFFSET( i ) ( ( char* ) NULL + ( i ) )

GeomMesh::GeomMesh( int faceCount, int vertexCount, VertexData* data, GLuint* indices )
	: faceCount(faceCount),
	  vertexCount( vertexCount )
{
	this->dataVBO = 0;
	this->indexVBO = 0;

	this->data = data;
	this->indexBuffer = indices;
}

GeomMesh::~GeomMesh()
{
	if ( this->dataVBO )
		glDeleteBuffers( 1, &this->dataVBO );

	if ( this->indexVBO )
		glDeleteBuffers( 1, &this->indexVBO );

	if ( this->data )
		delete[] this->data;

	if ( this->indexBuffer )
		delete[] this->indexBuffer;
}

bool
GeomMesh::render()
{
	GLint status;

	//GeomType::render();
	
	// lazy loading
	if ( 0 == this->dataVBO )
	{
		// generate and setup data vbo
		glGenBuffers( 1, &this->dataVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glGenBuffers failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ARRAY_BUFFER, this->dataVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBufferData( GL_ARRAY_BUFFER, sizeof( VertexData ) * this->vertexCount, this->data, GL_STATIC_DRAW );
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
		glGenBuffers( 1, &this->indexVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glGenBuffers indices failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->indexVBO );
		status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			cout << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER failed: " << gluErrorString( status )  << endl;
			return false;
		}

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->faceCount * 3, this->indexBuffer, GL_STATIC_DRAW );
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

	glBindBuffer( GL_ARRAY_BUFFER, this->dataVBO );
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
		cout << "GeomMesh::render: glEnableVertexAttribArray(0) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glEnableVertexAttribArray( 1 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glEnableVertexAttribArray(1) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 0 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glVertexAttribPointer(0) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 12 ) );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glVertexAttribPointer(1) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->indexVBO );
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

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glBindBuffer(0) GL_ELEMENT_ARRAY_BUFFER - render - failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glBindBuffer(0) GL_ELEMENT_ARRAY_BUFFER - render - failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glDisableVertexAttribArray( 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDisableVertexAttribArray(1) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	glDisableVertexAttribArray( 1 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "GeomMesh::render: glDisableVertexAttribArray(1) failed: " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}
