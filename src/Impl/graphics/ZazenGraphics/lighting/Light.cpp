#include "Light.h"

#include <string.h>
#include <iostream>

using namespace std;

Light*
Light::createSpot( float fov, int width, int height, bool shadowCaster )
{
	Light* light = new Light( width, height, Light::SPOT, shadowCaster );
	if ( false == light->createShadowMap() )
	{
		delete light;
		return NULL;
	}
	
	light->setFov( fov );
	light->setupPerspective();

	return light;
}

Light*
Light::createDirectional( int width, int height, bool shadowCaster )
{
	Light* light = new Light( width, height, Light::DIRECTIONAL, shadowCaster );
	if ( false == light->createShadowMap() )
	{
		delete light;
		return NULL;
	}
	
	light->setupOrtho();

	return light;
}

Light*
Light::createPoint( int width, bool shadowCaster )
{
	Light* light = new Light( width, width, Light::POINT, shadowCaster );
	if ( false == light->createShadowMap() )
	{
		delete light;
		return NULL;
	}
	
	// point-light has always field of view of 90 degrees per face (need to cover 360 degrees)
	light->setFov( 90.0f );
	light->setupPerspective();

	return light;
}

Light::Light( int width, int height, LightType type, bool shadowCaster )
	: Viewer( width, height ),
	  m_type( type ),
	  m_shadowCaster( shadowCaster )
{
	this->m_falloff = 0.0f;

	this->m_boundingMesh = NULL;
	this->m_shadowMap = NULL;
}

Light::~Light()
{
	RenderTarget::destroy( this->m_shadowMap );
}

bool
Light::createShadowMap()
{
	if ( this->m_shadowCaster )
	{
		RenderTarget* shadowMap = NULL;

		if ( LightType::POINT == this->m_type )
		{
			// point-lights need a cube shadow-map
			shadowMap = RenderTarget::create( this->getWidth(), this->getHeight(), RenderTarget::RT_SHADOW_CUBE );
		}
		else
		{
			shadowMap = RenderTarget::create( this->getWidth(), this->getHeight(), RenderTarget::RT_SHADOW_PLANAR );
		}

		if ( NULL == shadowMap )
		{
			return false;
		}

		this->m_shadowMap = shadowMap;
	}

	return true;
}
