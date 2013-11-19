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

		// cube vertices for vertex buffer object
		GLfloat cube_vertices[] = {
		  -1.0,  1.0,  1.0,
		  -1.0, -1.0,  1.0,
		   1.0, -1.0,  1.0,
		   1.0,  1.0,  1.0,
		  -1.0,  1.0, -1.0,
		  -1.0, -1.0, -1.0,
		   1.0, -1.0, -1.0,
		   1.0,  1.0, -1.0,
		};

		glGenBuffers( 1, &SkyBox::instance->m_dataVBO );
		GL_PEEK_ERRORS_AT
		glBindBuffer( GL_ARRAY_BUFFER, SkyBox::instance->m_dataVBO );
		GL_PEEK_ERRORS_AT
		glBufferData( GL_ARRAY_BUFFER, sizeof( cube_vertices ), cube_vertices, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT

		// cube indices for index buffer object
		GLint cube_indices[] = {
		  0, 1, 2, 3,
		  3, 2, 6, 7,
		  7, 6, 5, 4,
		  4, 5, 1, 0,
		  0, 3, 7, 4,
		  1, 2, 6, 5,
		};

		glGenBuffers( 1, &SkyBox::instance->m_indexVBO );
		GL_PEEK_ERRORS_AT
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, SkyBox::instance->m_indexVBO );
		GL_PEEK_ERRORS_AT
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( cube_indices ), cube_indices, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT
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
	this->m_cubeMap = NULL;

	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	SkyBox::instance = this;
}

SkyBox::~SkyBox()
{
	if ( this->m_dataVBO )
	{
		glDeleteBuffers( 1, &this->m_dataVBO );
	}

	if ( this->m_indexVBO )
	{
		glDeleteBuffers( 1, &this->m_indexVBO );
	}

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
	
	this->m_cubeMap->bind( 0 );

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG
	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG
	glDrawElements( GL_QUADS, 24, GL_UNSIGNED_INT, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glDisableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	GL_PEEK_ERRORS_AT_DEBUG
	glEnable( GL_CULL_FACE );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}
