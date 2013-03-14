/*
 *  spacebox.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GeomSkyBox.h"

using namespace std;

#define BOX_SIDE_SIZE 100

GeomSkyBox::GeomSkyBox(Viewer& cam, string& folder)
	: cam(cam)
{
	this->east = Texture::get(folder + "/east.png");
	this->west = Texture::get(folder + "/west.png");
	this->up = Texture::get(folder + "/up.png");
	this->down = Texture::get(folder + "/down.png");
	this->south = Texture::get(folder + "/south.png");
	this->north = Texture::get(folder + "/north.png");
}

GeomSkyBox::~GeomSkyBox()
{
}

bool
GeomSkyBox::render()
{
//	GeomType::render();
/*
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

	float xPos = this->cam.viewingMatrix[12];
	float yPos = this->cam.viewingMatrix.data[13];
	float zPos = this->cam.viewingMatrix.data[14];

	this->cam.viewingMatrix.data[12] = 0;
	this->cam.viewingMatrix.data[13] = 0;
	this->cam.viewingMatrix.data[14] = 0;

	glLoadMatrixf(this->cam.viewingMatrix.data);

	// Front Face
	this->east->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
	glEnd();
	this->east->deactivate();

	// Back Face
	this->west->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
	glEnd();
	this->west->deactivate();

	// Top Face
	this->up->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
	glEnd();
	this->up->deactivate();

	// Bottom Face
	this->down->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
	glEnd();
	this->down->deactivate();

	// Right face
	this->south->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
	glEnd();
	this->south->deactivate();

	// Left Face
	this->north->activate();
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-BOX_SIDE_SIZE, -BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE,  BOX_SIDE_SIZE);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-BOX_SIDE_SIZE,  BOX_SIDE_SIZE, -BOX_SIDE_SIZE);	// Top Left Of The Texture and Quad
	glEnd();
	this->north->deactivate();

	this->cam.viewingMatrix.data[12] = xPos;
	this->cam.viewingMatrix.data[13] = yPos;
	this->cam.viewingMatrix.data[14] = zPos;

	glLoadMatrixf(this->cam.viewingMatrix.data);

	// activate z-buffering and face culling
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
*/

	return true;
}
