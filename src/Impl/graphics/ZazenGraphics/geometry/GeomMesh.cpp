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
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glGenBuffers failed";
			return false;
		}

		glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed";
			return false;
		}

		glBufferData( GL_ARRAY_BUFFER, sizeof( VertexData ) * this->vertexCount, this->m_vertexData, GL_STATIC_DRAW );
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBufferData GL_ARRAY_BUFFER failed";
			return false;
		}

		// generate and setup index vbo
		glGenBuffers( 1, &this->m_indexVBO );
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glGenBuffers indices failed";
			return false;
		}

		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER failed";
			return false;
		}

		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * this->faceCount * 3, this->m_indexBuffer, GL_STATIC_DRAW );
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBufferData GL_ELEMENT_ARRAY_BUFFER failed";
			return false;
		}
	}

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ARRAY_BUFFER failed";
		return false;
	}
#endif

	glEnableVertexAttribArray( 0 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glEnableVertexAttribArray( 0 ) failed";
		return false;
	}
#endif

	glEnableVertexAttribArray( 1 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glEnableVertexAttribArray( 1 ) failed";
		return false;
	}
#endif

	glEnableVertexAttribArray( 2 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glEnableVertexAttribArray( 2 ) failed";
		return false;
	}
#endif

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 0 ) );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glVertexAttribPointer( 0 ) failed";
		return false;
	}
#endif

	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 12 ) );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glVertexAttribPointer( 1 ) failed";
		return false;
	}
#endif

	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ), BUFFER_OFFSET( 24 ) );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glVertexAttribPointer( 2 ) failed";
		return false;
	}
#endif

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glBindBuffer GL_ELEMENT_ARRAY_BUFFER - render - failed";
		return false;
	}
#endif

	glDrawElements( GL_TRIANGLES, this->faceCount * 3, GL_UNSIGNED_INT, BUFFER_OFFSET( 0 ) );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glDrawElements failed";
		return false;
	}
#endif

	glDisableVertexAttribArray( 0 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glDisableVertexAttribArray( 0 ) failed";
		return false;
	}
#endif
	
	glDisableVertexAttribArray( 1 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glDisableVertexAttribArray( 1 ) failed";
		return false;
	}
#endif

	glDisableVertexAttribArray( 2 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeomMesh::render: glDisableVertexAttribArray( 2 ) failed";
		return false;
	}
#endif

	return true;
}
