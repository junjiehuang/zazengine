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
using namespace boost;

map<string, Texture*> Texture::allTextures;
boost::filesystem::path Texture::textureDataPath;

void
Texture::init( const boost::filesystem::path& textureDataPath )
{
	ilInit();
	iluInit();
	ilutInit();

	Texture::textureDataPath = textureDataPath;
}

Texture*
Texture::get( const std::string& file )
{
	map<string, Texture*>::iterator findIter = Texture::allTextures.find( file );
	if ( findIter != Texture::allTextures.end() )
		return findIter->second;
	
	filesystem::path fullFileName( Texture::textureDataPath.generic_string() + file );

	if ( false == filesystem::exists( fullFileName ) )
	{
		cout << "ERROR ... in Texture::get: file for texture " << fullFileName << " does not exist" << endl;
		return NULL;
	}

	if ( filesystem::is_directory( fullFileName ) )
	{
		cout << "ERROR ... in Texture::get: file for texture " << fullFileName << " is a directory" << endl;
		return NULL;
	}

	GLuint textureID = Texture::createGLTexture( fullFileName );
	if ( 0 == textureID )
	{
		return NULL;
	}

	cout << "INFO ... successfully loaded texture from " << fullFileName << endl;

	Texture* newTexture = new Texture( textureID );
	
	Texture::allTextures[ file ] = newTexture;
	
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
	this->m_textureID = texID;
}

Texture::~Texture()
{
	glDeleteTextures( 1, &this->m_textureID );
}

void
Texture::bind( int textureUnit )
{
	GLenum status;

	glActiveTexture( GL_TEXTURE0 + textureUnit );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR ... in Texture::bind: failed glActiveTexture with " << gluErrorString( status ) << endl;
		return;
	}

	glBindTexture( GL_TEXTURE_2D, this->m_textureID );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR ... in Texture::bind: failed glBindTexture with " << gluErrorString( status ) << endl;
		return;
	}
}

GLuint
Texture::createGLTexture( const filesystem::path& fileName )
{
	ILuint imageID;				// Create a image ID as a ULuint
	GLuint textureID;			// Create a texture ID as a GLuint
	ILboolean success;			// Create a flag to keep track of success/failure
	ILenum error;				// Create a flag to keep track of the IL error state

	ilGenImages( 1, &imageID ); 		// Generate the image ID
	ilBindImage( imageID ); 			// Bind the image

	success = ilLoadImage( fileName.generic_string().c_str() ); 	// Load the image file

	// If we managed to load the image, then we can start to do things with it...
	if ( success )
	{
		// If the image is flipped (i.e. upside-down and mirrored, flip it the right way up!)
		ILinfo ImageInfo;
		iluGetImageInfo( &ImageInfo );
		if ( IL_ORIGIN_UPPER_LEFT == ImageInfo.Origin )
		{
			iluFlipImage();
		}

		// ... then attempt to conver it.
		// NOTE: If your image contains alpha channel you can replace IL_RGB with IL_RGBA
		success = ilConvertImage( IL_RGB, IL_UNSIGNED_BYTE );

		// Quit out if we failed the conversion
		if ( !success )
		{
			error = ilGetError();
			cout << "ERROR ... in Texture::createGLTexture: Image load failed - IL reports error: " << error << endl;
			return 0;
		}

		// Generate a new texture
		glGenTextures( 1, &textureID );

		// Bind the texture to a name
		glBindTexture( GL_TEXTURE_2D, textureID );

		// Set texture clamping method
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

		// Set texture interpolation method to use linear interpolation (no MIPMAPS)
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		
		// Specify the texture specification
		glTexImage2D(GL_TEXTURE_2D, 				// Type of texture
					 0,				// Pyramid level (for mip-mapping) - 0 is the top level
					 ilGetInteger( IL_IMAGE_BPP ),	// Image colour depth
					 ilGetInteger( IL_IMAGE_WIDTH ),	// Image width
					 ilGetInteger( IL_IMAGE_HEIGHT ),	// Image height
					 0,				// Border width in pixels (can either be 1 or 0)
					 ilGetInteger( IL_IMAGE_FORMAT ),	// Image format (i.e. RGB, RGBA, BGR etc.) GL_RGB
					 GL_UNSIGNED_BYTE,		// Image data type
					 ilGetData() );			// The actual image data itself

		// Bind the texture to a name
		glBindTexture( GL_TEXTURE_2D, 0 );
 	}
  	else // If we failed to open the image file in the first place...
  	{
		error = ilGetError();
		cout << "ERROR ... in Texture::createGLTexture: Image load failed - IL reports error: " << error << endl;
		return 0;
  	}

 	ilDeleteImages( 1, &imageID ); // Because we have already copied image data into texture data we can release memory used by image.

	return textureID; // Return the GLuint to the texture so you can use it!
}
