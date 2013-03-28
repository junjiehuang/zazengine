/*
 *  spacebox.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GeomSkyBox.h"

#include "../ZazenGraphics.h"
#include <glm/gtc/type_ptr.hpp>

using namespace std;

#define BOX_SIDE_SIZE 100

GeomSkyBox* GeomSkyBox::instance = NULL;

bool
GeomSkyBox::initialize( const boost::filesystem::path& textureFolder, const std::string& format )
{
	if ( NULL == GeomSkyBox::instance )
	{
		new GeomSkyBox();

		GeomSkyBox::instance->m_front = Texture::get( textureFolder.generic_string() + "/front." + format );
		if ( NULL == GeomSkyBox::instance->m_front )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get front-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_back = Texture::get( textureFolder.generic_string() + "/back." + format );
		if ( NULL == GeomSkyBox::instance->m_back )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get back-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_left = Texture::get( textureFolder.generic_string() + "/left." + format );
		if ( NULL == GeomSkyBox::instance->m_left )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get left-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_right = Texture::get( textureFolder.generic_string() + "/right." + format );
		if ( NULL == GeomSkyBox::instance->m_right )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get right-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_up = Texture::get( textureFolder.generic_string() + "/up." + format );
		if ( NULL == GeomSkyBox::instance->m_up )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get up-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_down = Texture::get( textureFolder.generic_string() + "/down." + format );
		if ( NULL == GeomSkyBox::instance->m_down )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get down-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}
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
	this->m_front = NULL;
	this->m_back = NULL;
	this->m_left = NULL;
	this->m_right = NULL;
	this->m_up = NULL;
	this->m_down = NULL;

	GeomSkyBox::instance = this;
}

GeomSkyBox::~GeomSkyBox()
{
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
	glm::mat4 modelViewMat = cam.m_viewMatrix;
	modelViewMat[ 3 ][ 0 ] = 0.0f;
	modelViewMat[ 3 ][ 1 ] = 0.0f;
	modelViewMat[ 3 ][ 2 ] = 0.0f;

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( projMat ) );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( modelViewMat ) );

	// Front Face
	this->m_front->bind( 0 );

	glBegin( GL_QUADS );
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  -BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad

		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  -BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad

		glTexCoord2f( 0.0f, 1.0f );  glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
		
	glEnd();

	this->m_front->unbind();
	/////////////////////////

	// Back Face
	this->m_back->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad

		glTexCoord2f( 0.0f, 1.0f );  glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
	glEnd();

	this->m_back->unbind();
	/////////////////////////

	// Top Face
	this->m_up->bind( 0 );
	glBegin( GL_QUADS );
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f );  glVertex3f( BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
	glEnd();

	this->m_up->unbind();
	/////////////////////////

	// Bottom Face
	this->m_down->bind( 0 );

	glBegin( GL_QUADS );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
		
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad
	glEnd();

	this->m_down->unbind();
	/////////////////////////

	// Right face
	this->m_right->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE,  -BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 1.0f );  glVertex3f( BOX_SIDE_SIZE, BOX_SIDE_SIZE,  -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
	glEnd();

	this->m_right->unbind();
	/////////////////////////

	// Left Face
	this->m_left->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE, BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
	glEnd();

	this->m_left->unbind();
	/////////////////////////

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );

	return true;
}
