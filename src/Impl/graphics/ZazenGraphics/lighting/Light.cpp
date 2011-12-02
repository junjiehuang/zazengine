/*
 * Light.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: jonathan
 */

#include "Light.h"

#include <string.h>
#include <iostream>

using namespace std;

Light*
Light::createSpoptLight( float fov, int width, int height )
{
	Light* light = new Light( width, height, Light::SPOT );
	if ( false == light->createShadowMap( width, height ) )
	{
		delete light;
		light = 0;
	}
	else
	{
		light->setFov( fov );
		light->setupPerspective();
	}

	return light;
}

Light*
Light::createDirectionalLight( int width, int height )
{
	Light* light = new Light( width, height, Light::DIRECTIONAL );
	if ( false == light->createShadowMap( width, height ) )
	{
		delete light;
		light = 0;
	}
	else
	{
		light->setupOrtho();
	}

	return light;
}

Light*
Light::createPointLight( int side )
{
	Light* light = new Light( side, side, Light::POINT );
	if ( false == light->createShadowCubeMap( side ) )
	{
		delete light;
		light = 0;
	}
	else
	{
		light->setFov( 90 );
		light->setupPerspective();
	}

	return light;
}

Light::Light( int width, int height, LightType type )
	: Viewer( width, height ),
	  m_type( type )
{
	this->m_shadowMap = 0;
	memset( this->m_cubeShadowMap, 0, sizeof( this->m_cubeShadowMap ) );

	this->m_shadowCaster = true;

	this->m_falloff = 0.0f;
}

Light::~Light()
{
	if ( this->m_shadowMap )
		glDeleteTextures( 1, &this->m_shadowMap );

	if ( this->m_cubeShadowMap[ 0 ] )
		glDeleteTextures( 6, this->m_cubeShadowMap );
}

bool
Light::createShadowMap( int width, int height )
{
	GLenum status;
	GLuint shadowMapID;

	// Try to use a texture depth component
	glGenTextures( 1, &shadowMapID );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in Light::createShadowMap: glGenTextures failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	glBindTexture( GL_TEXTURE_2D, shadowMapID );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in Light::createShadowMap: glBindTexture failed with " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	// GL_LINEAR does not make sense for depth texture. However, next tutorial shows usage of GL_LINEAR and PCF
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );

	// Remove artifact on the edges of the shadowmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

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

	// unbind framebuffer depth-target
	glBindTexture( GL_TEXTURE_2D, 0 );

	this->m_shadowMap = shadowMapID;

	return true;
}

bool
Light::createShadowCubeMap( int side )
{
	return false;
}
