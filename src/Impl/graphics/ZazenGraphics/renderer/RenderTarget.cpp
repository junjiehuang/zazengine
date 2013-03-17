#include "RenderTarget.h"

#include <iostream>

using namespace std;

RenderTarget*
RenderTarget::create( GLsizei width, GLsizei height, RenderTargetType targetType )
{
	GLuint id = 0;
	GLenum status;

	glGenTextures( 1, &id );
	if ( GL_NO_ERROR != ( status = glGetError() )  )
	{
		cout << "ERROR in RenderTarget::create: glGenTextures failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glBindTexture( GL_TEXTURE_2D, id );
	if ( GL_NO_ERROR != ( status = glGetError() )  )
	{
		cout << "ERROR in RenderTarget::create: glBindTexture failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );

	// Remove artifact on the edges of the shadowmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

	if ( RenderTarget::RT_DEPTH == targetType )
	{
		glTexParameteri( GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE );

		// for now we create shadowmaps in same width and height as their viewing frustum
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0 );
		if ( GL_NO_ERROR != ( status = glGetError() ) )
		{
			cout << "ERROR in Light::createShadowMap: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}
	else if ( RenderTarget::RT_SHADOW == targetType )
	{

		// need to enable comparison-mode for depth-texture to use it as a shadow2DSampler in shader
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL );

		// for now we create shadowmaps in same width and height as their viewing frustum
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0 );
		if ( GL_NO_ERROR != ( status = glGetError() ) )
		{
			cout << "ERROR in Light::createShadowMap: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}
	else
	{
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in RenderTarget::create: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}

	// unbind framebuffer depth-target
	glBindTexture( GL_TEXTURE_2D, 0 );

	return new RenderTarget( id, targetType );
}

bool
RenderTarget::destroy( RenderTarget* renderTarget )
{
	if ( NULL == renderTarget )
	{
		return true;
	}

	glDeleteTextures( 1, &renderTarget->m_id );
	delete renderTarget;

	return true;
}

RenderTarget::RenderTarget( GLuint id, RenderTargetType targetType )
{
	this->m_id = id;
	this->m_targetType = targetType;

	this->m_boundIndex = 0;
}

RenderTarget::~RenderTarget()
{
}

bool
RenderTarget::bind( unsigned int index )
{
	GLenum status;

	this->m_boundIndex = index;

	// bind diffuse rendering target to texture unit 0
	glActiveTexture( GL_TEXTURE0 + this->m_boundIndex );
	glBindTexture( GL_TEXTURE_2D, this->m_id );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in RenderTarget::bind: glBindTexture of mrt " << this->m_boundIndex << " failed with " << gluErrorString( status ) << endl;
		return false;
	}

	return true;
}

bool 
RenderTarget::unbind()
{
	GLenum status;

	// bind depth-buffer to texture-unit MRT_COUNT
	glActiveTexture( GL_TEXTURE0 + this->m_boundIndex );
	glBindTexture( GL_TEXTURE_2D, this->m_id );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in FrameBufferObject::bindAllTargets: glBindTexture of depth-buffer failed with " << gluErrorString( status ) << endl;
		return false;
	}


	return true;
}
