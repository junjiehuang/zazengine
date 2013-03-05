/*
 *  texture.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 30.04.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "Texture.h"

#include <SDL/SDL_image.h>

#include <iostream>

using namespace std;

map<string, Texture*> Texture::allTextures;

Texture*
Texture::load(const std::string& file)
{
	map<string, Texture*>::iterator findIter = Texture::allTextures.find(file);
	if ( findIter != Texture::allTextures.end() )
		return findIter->second;
	
	string fullFileName = "media/graphics/textures/" + file;
	
	GLuint textureID;
    SDL_Surface* surface = Texture::loadImage(fullFileName);
    GLenum textureFormat = GL_RGBA;
	GLint nOfColors;

	if ( surface )
	{
		// Check that the image's width is a power of 2
		if ((surface->w & (surface->w - 1)) != 0)
		{
			cout << "WARNING ... " << fullFileName << " width is not a power of 2" << endl;
		}
		
		// Also check if the height is a power of 2
		if ((surface->h & (surface->h - 1)) != 0)
		{
			cout << "WARNING ... " << fullFileName << " height is not a power of 2" << endl;;
		}
		
        // get the number of channels in the SDL surface
        nOfColors = surface->format->BytesPerPixel;
        if (nOfColors == 4)
        {
			if (surface->format->Rmask == 0x000000ff)
				textureFormat = GL_RGBA;
			else
				textureFormat = GL_BGRA;
        }
        else if (nOfColors == 3)
        {
			if (surface->format->Rmask == 0x000000ff)
				textureFormat = GL_RGB;
			else
				textureFormat = GL_BGR;
        }
        else
        {
        	cout << "WARNING ... the image is not truecolor..  this will probably break" << endl;
        }
        
        // generate texture id
		glGenTextures(1, &textureID);
		// bind texture
		glBindTexture(GL_TEXTURE_2D, textureID);
		// anisotropic filtering
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		
        // upload data
		glTexImage2D( GL_TEXTURE_2D, 0, nOfColors, surface->w, surface->h, 0, textureFormat, GL_UNSIGNED_BYTE, surface->pixels );
		// unbind texture
		glBindTexture(GL_TEXTURE_2D, 0);

	}
	else
	{
		cout << "ERROR ... SDL could not load texture " << fullFileName << " - failed with " << SDL_GetError() << endl;
		return 0;
	}    
	
	Texture* newTexture = new Texture(textureID, surface);
	
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

Texture::Texture(GLuint texID, SDL_Surface* surf)
{
	this->textureID = texID;
	this->surface = surf;

	this->textureUnit = -1;
}

Texture::~Texture()
{
	glDeleteTextures( 1, &this->textureID );
	SDL_FreeSurface( this->surface );
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

SDL_Surface*
Texture::loadImage(const std::string& filename)
{
    SDL_Surface* loadedImage = NULL;
    SDL_Surface* optimizedImage = NULL;
    
    loadedImage = IMG_Load(filename.c_str());
	
    if( loadedImage != NULL )
    {
        optimizedImage = SDL_DisplayFormat( loadedImage );
        SDL_FreeSurface( loadedImage );
    }

    return optimizedImage;
}
