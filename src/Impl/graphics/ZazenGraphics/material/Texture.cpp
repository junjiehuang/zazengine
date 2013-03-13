/*
 *  texture.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 30.04.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "Texture.h"

#include <IL/ilut.h>

#include <iostream>

using namespace std;

map<string, Texture*> Texture::allTextures;

Texture*
Texture::load( const std::string& file )
{
	map<string, Texture*>::iterator findIter = Texture::allTextures.find( file );
	if ( findIter != Texture::allTextures.end() )
		return findIter->second;
	
	string fullFileName = "media/graphics/textures/" + file;
	
	GLuint textureID = ilutGLLoadImage( ( ILstring ) fullFileName.c_str() );
	Texture* newTexture = new Texture( textureID );
	
	Texture::allTextures[file] = newTexture;
	
	return newTexture;
}

void
Texture::freeAll()
{
	map<string, Texture*>::iterator iter = Texture::allTextures.begin();
	while(iter != Texture::allTextures.end())
	{
		delete iter->second;
		
		iter++;
	}
	
	Texture::allTextures.clear();
}

Texture::Texture( GLuint texID )
{
	this->textureID = texID;

	this->textureUnit = -1;
}

Texture::~Texture()
{
	glDeleteTextures( 1, &this->textureID );
}

void
Texture::bind( int textureUnit )
{
	if ( this->textureUnit != - 1)
		return;

	this->textureUnit = textureUnit;

	glActiveTexture( GL_TEXTURE0 + this->textureUnit );
	glBindTexture(GL_TEXTURE_2D, this->textureID);
}

void Texture::unbind()
{
	if (this->textureUnit == -1)
		return;

	glActiveTexture( GL_TEXTURE0 + this->textureUnit );
	glBindTexture( GL_TEXTURE_2D, 0 );

	this->textureUnit = -1;
}
