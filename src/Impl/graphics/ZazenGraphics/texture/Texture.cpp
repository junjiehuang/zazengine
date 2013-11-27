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

bool
Texture::unbind()
{
	if ( -1 != this->m_boundToUnit )
	{
		if ( Texture::m_currentTextureUnit != this->m_boundToUnit )
		{
			glActiveTexture( GL_TEXTURE0 + this->m_boundToUnit );
			GL_PEEK_ERRORS_AT_DEBUG

			Texture::m_currentTextureUnit = this->m_boundToUnit;
		}

		if ( Texture::TEXTURE_2D == this->m_textureType ) 
		{
			glBindTexture( GL_TEXTURE_2D, 0 );
			GL_PEEK_ERRORS_AT_DEBUG

		}
		else if ( Texture::TEXTURE_CUBE == this->m_textureType ) 
		{
			glBindTexture( GL_TEXTURE_CUBE_MAP, 0 );
			GL_PEEK_ERRORS_AT_DEBUG
		}

		this->m_boundToUnit = -1;
	}

	return true;
}