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
	RenderTarget::destroy( this->m_shadowMap );

	if ( this->m_cubeShadowMap[ 0 ] )
	{
		glDeleteTextures( 6, this->m_cubeShadowMap );
	}
}

bool
Light::createShadowMap( int width, int height )
{
	RenderTarget* shadowMap = RenderTarget::create( width, height, RenderTarget::RT_SHADOW );
	if ( NULL == shadowMap )
	{
		return false;
	}

	this->m_shadowMap = shadowMap;

	return true;
}

bool
Light::createShadowCubeMap( int side )
{
	return false;
}
