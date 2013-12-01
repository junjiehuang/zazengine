/*
 *  SkyBox.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *
 */

#include "SkyBox.h"

#include "../Util/GLUtils.h"
#include "../Texture/TextureFactory.h"
#include "../Geometry/GeometryFactory.h"

#include "../ZazenGraphics.h"

#include <glm/gtc/type_ptr.hpp>

using namespace std;

SkyBox* SkyBox::instance = NULL;

bool
SkyBox::initialize( const boost::filesystem::path& textureFolder, const std::string& format )
{
	if ( NULL == SkyBox::instance )
	{
		new SkyBox();

		SkyBox::instance->m_cubeMap = TextureFactory::getCube( textureFolder, format );
		if ( NULL == SkyBox::instance->m_cubeMap )
		{
			ZazenGraphics::getInstance().getLogger().logError( "SkyBox::initialize: couldn't create cube-map" );
			SkyBox::shutdown();
			return false;
		}

		SkyBox::instance->m_cubeMesh = GeometryFactory::getRef().createUnitCube();
		if ( NULL == SkyBox::instance->m_cubeMesh )
		{
			ZazenGraphics::getInstance().getLogger().logError( "SkyBox::initialize: couldn't create cube-mesh" );
			SkyBox::shutdown();
			return false;
		}
	}

	return true;
}

bool
SkyBox::shutdown()
{
	if ( SkyBox::instance )
	{
		delete SkyBox::instance;
	}

	return true;
}

SkyBox::SkyBox()
{
	this->m_cubeMesh = NULL;
	this->m_cubeMap = NULL;

	SkyBox::instance = this;
}

SkyBox::~SkyBox()
{
	// NOTE: texture and mesh are owned by their factories

	SkyBox::instance = NULL;
}

bool
SkyBox::render( const Viewer& camera, UniformBlock* cameraBlock, UniformBlock* transformsBlock )
{
	// disable depth-writing, sky-box is ALWAYS behind everything else
	glDisable( GL_DEPTH_TEST );
	GL_PEEK_ERRORS_AT_DEBUG
	glDisable( GL_CULL_FACE );
	GL_PEEK_ERRORS_AT_DEBUG

	// force projective-projection in sky-box (if camera could have ortho)
	glm::mat4 projMat = camera.createPerspProj();
	// reset modelview back to origin, to stick with camera, sky-box is so far away that it doesn't move
	glm::mat4 modelViewMat = camera.getViewMatrix();
	modelViewMat[ 3 ][ 0 ] = 0.0f;
	modelViewMat[ 3 ][ 1 ] = 0.0f;
	modelViewMat[ 3 ][ 2 ] = 0.0f;

	if ( false == cameraBlock->bindBuffer() )
	{
		return false;
	}

	cameraBlock->updateField( "CameraUniforms.projectionMatrix", projMat );

	if ( false == transformsBlock->bindBuffer() )
	{
		return false;
	}

	transformsBlock->updateField( "TransformUniforms.modelViewMatrix", modelViewMat );
	
	// NOTE: need to bind cube-map to unit Texture::CUBE_RANGE_START because unit 0 is already occupied by 2D-textures during light-rendering - 
	// it is not allowed to bind different types of textures to the same unit
	this->m_cubeMap->bind( Texture::CUBE_RANGE_START );
	this->m_cubeMesh->render();

	// NOTE: need to unbind cube-map because unit 0 is already occupied by 2D-textures during light-rendering - it is not allowed to bind different types of textures to the same unit
	//this->m_cubeMap->unbind();

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	GL_PEEK_ERRORS_AT_DEBUG
	glEnable( GL_CULL_FACE );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}
