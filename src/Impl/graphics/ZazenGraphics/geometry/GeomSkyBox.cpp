/*
 *  spacebox.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GeomSkyBox.h"

#include "../Util/GLUtils.h"
#include "../Texture/TextureFactory.h"

#include "GeometryFactory.h"

#include "../ZazenGraphics.h"
#include <glm/gtc/type_ptr.hpp>

using namespace std;

GeomSkyBox* GeomSkyBox::instance = NULL;

bool
GeomSkyBox::initialize( const boost::filesystem::path& textureFolder, const std::string& format )
{
	if ( NULL == GeomSkyBox::instance )
	{
		new GeomSkyBox();

		GeomSkyBox::instance->m_cubeMap = TextureFactory::getCube( textureFolder, format );
		if ( NULL == GeomSkyBox::instance->m_cubeMap )
		{
			ZazenGraphics::getInstance().getLogger().logError( "GeomSkyBox::initialize: couldn't create cube-map" );
			GeomSkyBox::shutdown();
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

		glGenBuffers( 1, &GeomSkyBox::instance->m_dataVBO );
		GL_PEEK_ERRORS_AT
		glBindBuffer( GL_ARRAY_BUFFER, GeomSkyBox::instance->m_dataVBO );
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

		glGenBuffers( 1, &GeomSkyBox::instance->m_indexVBO );
		GL_PEEK_ERRORS_AT
		glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, GeomSkyBox::instance->m_indexVBO );
		GL_PEEK_ERRORS_AT
		glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( cube_indices ), cube_indices, GL_STATIC_DRAW );
		GL_PEEK_ERRORS_AT
	}

	return true;
}

bool
GeomSkyBox::shutdown()
{
	if ( GeomSkyBox::instance )
	{
		delete GeomSkyBox::instance;
	}

	return true;
}

GeomSkyBox::GeomSkyBox()
{
	this->m_cubeMap = NULL;

	this->m_dataVBO = 0;
	this->m_indexVBO = 0;

	GeomSkyBox::instance = this;
}

GeomSkyBox::~GeomSkyBox()
{
	if ( this->m_dataVBO )
	{
		glDeleteBuffers( 1, &this->m_dataVBO );
	}

	if ( this->m_indexVBO )
	{
		glDeleteBuffers( 1, &this->m_indexVBO );
	}

	GeomSkyBox::instance = NULL;
}

bool
GeomSkyBox::render()
{
	Viewer& cam = ZazenGraphics::getInstance().getCamera();

	// disable depth-writing, sky-box is ALWAYS behind everything else
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_CULL_FACE );

	// force projective-projection in sky-box (camera could have ortho)
	glm::mat4 projMat = ZazenGraphics::getInstance().getCamera().createPerspProj();
	// reset modelview back to origin, to stick with camera, sky-box is so far away that it doesn't move
	glm::mat4 modelViewMat = cam.getViewMatrix();
	modelViewMat[ 3 ][ 0 ] = 0.0f;
	modelViewMat[ 3 ][ 1 ] = 0.0f;
	modelViewMat[ 3 ][ 2 ] = 0.0f;

	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	if ( false == this->m_transformsBlock->updateField( "TransformUniforms.modelViewMatrix", modelViewMat ) )
	{
		return false;
	}

	if ( false == this->m_transformsBlock->updateField( "TransformUniforms.projectionMatrix", projMat ) )
	{
		return false;
	}

	this->m_cubeMap->bind( 0 );

	glBindBuffer( GL_ARRAY_BUFFER, this->m_dataVBO );
	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, this->m_indexVBO );
	glDrawElements( GL_QUADS, 24, GL_UNSIGNED_INT, 0 );

	glDisableVertexAttribArray( 0 );

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );

	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}
