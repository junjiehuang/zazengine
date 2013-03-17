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

		GeomSkyBox::instance->east = Texture::get( textureFolder.generic_string() + "/east." + format );
		if ( NULL == GeomSkyBox::instance->east )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get east-texture" << endl;
			return false;
		}

		GeomSkyBox::instance->west = Texture::get( textureFolder.generic_string() + "/west." + format );
		if ( NULL == GeomSkyBox::instance->west )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get west-texture" << endl;
			return false;
		}

		GeomSkyBox::instance->up = Texture::get( textureFolder.generic_string() + "/up." + format );
		if ( NULL == GeomSkyBox::instance->up )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get up-texture" << endl;
			return false;
		}

		GeomSkyBox::instance->down = Texture::get( textureFolder.generic_string() + "/down." + format );
		if ( NULL == GeomSkyBox::instance->down )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get down-texture" << endl;
			return false;
		}

		GeomSkyBox::instance->south = Texture::get( textureFolder.generic_string() + "/south." + format );
		if ( NULL == GeomSkyBox::instance->south )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get south-texture" << endl;
			return false;
		}

		GeomSkyBox::instance->north = Texture::get( textureFolder.generic_string() + "/north." + format );
		if ( NULL == GeomSkyBox::instance->north )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get north-texture" << endl;
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
	this->east = NULL;
	this->west = NULL;
	this->up = NULL;
	this->down = NULL;
	this->south = NULL;
	this->north = NULL;

	GeomSkyBox::instance = this;
}

GeomSkyBox::~GeomSkyBox()
{
	this->east = NULL;
	this->west = NULL;
	this->up = NULL;
	this->down = NULL;
	this->south = NULL;
	this->north = NULL;

	GeomSkyBox::instance = NULL;
}

bool
GeomSkyBox::render()
{
	Viewer& cam = ZazenGraphics::getInstance().getCamera();
	float* data = glm::value_ptr( cam.m_viewMatrix );

	glDisable( GL_DEPTH_TEST );
	glDisable( GL_CULL_FACE );

	float xPos = data[ 12 ];
	float yPos = data[ 13 ];
	float zPos = data[ 14 ];

	data[ 12 ] = 0.0f;
	data[ 13 ] = 0.0f;
	data[ 14 ] = 0.0f;

	glLoadMatrixf( data );

	// Front Face
	this->east->bind( 0 );

	glBegin( GL_QUADS );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f );  glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
		
	glEnd();

	this->east->unbind();
	/////////////////////////

	// Back Face
	this->west->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f );  glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
	glEnd();

	this->west->unbind();
	/////////////////////////

	// Top Face
	this->up->bind( 0 );
	glBegin( GL_QUADS );
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 1.0f );  glVertex3f( BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
	glEnd();

	this->up->unbind();
	/////////////////////////

	// Bottom Face
	this->down->bind( 0 );

	glBegin( GL_QUADS );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
		
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad
	glEnd();

	this->down->unbind();
	/////////////////////////

	// Right face
	this->south->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad
		
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad

		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 1.0f );  glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
	glEnd();

	this->south->unbind();
	/////////////////////////

	// Left Face
	this->north->bind( 0 );

	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Bottom Left Of The Texture and Quad
		 
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Bottom Right Of The Texture and Quad

		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE );	// Top Right Of The Texture and Quad
		 
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE );	// Top Left Of The Texture and Quad
	glEnd();

	this->north->unbind();
	/////////////////////////

	data[ 12 ] = xPos;
	data[ 13 ] = yPos;
	data[ 14 ] = zPos;

	glLoadMatrixf( data );

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );

	return true;
}
