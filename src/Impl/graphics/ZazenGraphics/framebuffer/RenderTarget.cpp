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
		return NULL;
	}

	glBindTexture( GL_TEXTURE_2D, id );
	if ( GL_NO_ERROR != ( status = glGetError() )  )
	{
		cout << "ERROR in RenderTarget::create: glBindTexture failed with " << gluErrorString( status ) << " - exit" << endl;
		return NULL;
	}

	RenderTarget* renderTarget = NULL;

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
			return NULL;
		}

		renderTarget = new RenderTarget( id, width, height, targetType );
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
			return NULL;
		}

		renderTarget = new RenderTarget( id, width, height, targetType );
		RenderTarget::m_shadowMapPool.push_back( renderTarget );
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
			return NULL;
		}

		renderTarget = new RenderTarget( id, width, height, targetType );
	}
	else
	{
		cout << "ERROR in RenderTarget::create: unsupported RenderTarget-Type - exit" << endl;
		return NULL;
	}

	return renderTarget;
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

	RenderTarget::m_shadowMapPool.clear();
}

RenderTarget::RenderTarget( GLuint id, GLsizei width, GLsizei height, RenderTargetType targetType )
	: Texture( id, Texture::TEXTURE_2D )
{
	this->m_targetType = targetType;

	this->m_width = width;
	this->m_height = height;
}

RenderTarget::~RenderTarget()
{
}
