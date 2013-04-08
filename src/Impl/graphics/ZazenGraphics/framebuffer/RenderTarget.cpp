#include "RenderTarget.h"

#include <iostream>

using namespace std;

vector<RenderTarget*>
RenderTarget::m_shadowMapPool;

RenderTarget*
RenderTarget::create( GLsizei width, GLsizei height, RenderTargetType targetType )
{
	GLuint id = 0;
	GLenum status;

	// use shadow-map pooling: only create maps with different resolution and reuse them
	if ( RenderTarget::RT_SHADOW == targetType )
	{
		RenderTarget* pooledShadowMap = RenderTarget::findShadowMapInPool( width, height );
		if ( NULL != pooledShadowMap )
		{
			return pooledShadowMap;
		}
	}

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

	if ( RenderTarget::RT_DEPTH == targetType )
	{
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );

		// Remove artifact on the edges of the shadowmap
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );

		glTexParameteri( GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY );

		// for now we create shadowmaps in same width and height as their viewing frustum
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL );
		if ( GL_NO_ERROR != ( status = glGetError() ) )
		{
			cout << "ERROR in Light::createShadowMap: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}
	else if ( RenderTarget::RT_SHADOW == targetType )
	{
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );

		// need to enable comparison-mode for depth-texture to use it as a shadow2DSampler in shader		
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE );

		glTexParameteri( GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY );

		// for now we create shadowmaps in same width and height as their viewing frustum
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL );
		if ( GL_NO_ERROR != ( status = glGetError() ) )
		{
			cout << "ERROR in Light::createShadowMap: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}
	else if ( RenderTarget::RT_COLOR == targetType )
	{
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );

		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL );
		if ( GL_NO_ERROR != ( status = glGetError() )  )
		{
			cout << "ERROR in RenderTarget::create: glTexImage2D failed with " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}
	}
	else
	{
		cout << "ERROR in RenderTarget::create: unsupported RenderTarget-Type - exit" << endl;
		return false;
	}

	// unbind framebuffer depth-target
	glBindTexture( GL_TEXTURE_2D, 0 );

	return new RenderTarget( id, width, height, targetType );
}

RenderTarget*
RenderTarget::findShadowMapInPool( GLsizei width, GLsizei height )
{
	for ( unsigned int i = 0; i < RenderTarget::m_shadowMapPool.size(); i++ )
	{
		RenderTarget* shadowMap = RenderTarget::m_shadowMapPool[ i ];
		if ( shadowMap->getWidth() == width && shadowMap->getHeight() == height )
		{
			return shadowMap;
		}
	}

	return NULL;
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

void
RenderTarget::cleanup()
{
	for ( unsigned int i = 0; i < RenderTarget::m_shadowMapPool.size(); i++ )
	{
		RenderTarget* shadowMap = RenderTarget::m_shadowMapPool[ i ];
		RenderTarget::destroy( shadowMap );
	}
}

RenderTarget::RenderTarget( GLuint id, GLsizei width, GLsizei height, RenderTargetType targetType )
{
	this->m_id = id;
	this->m_targetType = targetType;

	this->m_width = width;
	this->m_height = height;
}

RenderTarget::~RenderTarget()
{
}

bool
RenderTarget::bind( unsigned int index )
{
	GLenum status;

	glActiveTexture( GL_TEXTURE0 + index );
#ifdef CHECK_GL_ERROR
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in RenderTarget::bind: glActiveTexture of GL_TEXTURE" << index << " failed with " << gluErrorString( status ) << endl;
		return false;
	}
#endif

	glBindTexture( GL_TEXTURE_2D, this->m_id );

#ifdef CHECK_GL_ERROR
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in RenderTarget::bind: glBindTexture with id " << this->m_id << " failed with " << gluErrorString( status ) << endl;
		return false;
	}
#endif

	return true;
}
