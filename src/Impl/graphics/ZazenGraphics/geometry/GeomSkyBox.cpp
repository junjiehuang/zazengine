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

GeomSkyBox::GeomSkyBox()
{
	this->east = Texture::get( "east.jpg" );
	this->west = Texture::get( "west.jpg" );
	this->up = Texture::get( "up.jpg" );
	this->down = Texture::get( "down.jpg" );
	this->south = Texture::get( "south.jpg" );
	this->north = Texture::get( "north.jpg" );
}

GeomSkyBox::~GeomSkyBox()
{
}

bool
GeomSkyBox::render()
{
//	GeomType::render();
	Viewer& cam = ZazenGraphics::getInstance().getCamera();
	float* data = glm::value_ptr( cam.m_viewMatrix );

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

	float xPos = data[ 12 ];
	float yPos = data[ 13 ];
	float zPos = data[ 14 ];

	data[ 12 ] = 0.0f;
	data[ 13 ] = 0.0f;
	data[ 14 ] = 0.0f;

	glLoadMatrixf( data );

	// Front Face
	this->east->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
	glEnd();
	this->east->unbind();

	// Back Face
	this->west->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
	glEnd();
	this->west->unbind();

	// Top Face
	this->up->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
	glEnd();
	this->up->unbind();

	// Bottom Face
	this->down->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
	glEnd();
	this->down->unbind();

	// Right face
	this->south->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
	glEnd();
	this->south->unbind();

	// Left Face
	this->north->bind( 0 );
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
	glEnd();
	this->north->unbind();

	data[ 12 ] = xPos;
	data[ 13 ] = yPos;
	data[ 14 ] = zPos;

	glLoadMatrixf( data );

	// activate z-buffering and face culling
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	return true;
}
