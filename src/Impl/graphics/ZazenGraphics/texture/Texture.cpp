#include "Texture.h"

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"

#include <iostream>

using namespace std;

GLint Texture::m_currentTextureUnit = -1;

Texture::Texture( GLuint texID, TextureType type )
{
	this->m_id = texID;
	this->m_textureType = type;

	this->m_boundToUnit = -1;
}

Texture::~Texture()
{
	glDeleteTextures( 1, &this->m_id );
}

bool
Texture::bind( int textureUnit )
{
#ifdef _DEBUG
	// check if 2D-textures are bound to units within their supposed range ( < Texture::CUBE_RANGE_START )
	if ( Texture::TEXTURE_2D == this->m_textureType ) 
	{
		if ( Texture::CUBE_RANGE_START <= textureUnit )
		{
			ZazenGraphics::getInstance().getLogger().logWarning() << "Texture::bind binding 2D-Texture in Cube-Texture range! Unit is " << textureUnit;
		}
	}
	// check if Cube-textures are bound to units within their supposed range ( >= Texture::CUBE_RANGE_START )
	else if ( Texture::TEXTURE_CUBE == this->m_textureType ) 
	{
		if ( Texture::CUBE_RANGE_START > textureUnit  )
		{
			ZazenGraphics::getInstance().getLogger().logWarning() << "Texture::bind binding Cube-Texture in 2D-Texture range! Unit is " << textureUnit;
		}
	}
#endif

	if ( Texture::m_currentTextureUnit != textureUnit )
	{
		glActiveTexture( GL_TEXTURE0 + textureUnit );
		GL_PEEK_ERRORS_AT_DEBUG

		Texture::m_currentTextureUnit = textureUnit;
	}

	if ( Texture::TEXTURE_2D == this->m_textureType ) 
	{
		glBindTexture( GL_TEXTURE_2D, this->m_id );
		GL_PEEK_ERRORS_AT_DEBUG
	}
	else if ( Texture::TEXTURE_CUBE == this->m_textureType ) 
	{
		glBindTexture( GL_TEXTURE_CUBE_MAP, this->m_id );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	this->m_boundToUnit = textureUnit;

	return true;
}
