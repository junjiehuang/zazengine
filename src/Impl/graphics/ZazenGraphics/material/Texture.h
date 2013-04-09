/*
 *  texture.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 30.04.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _TEXTURE_H_
#define _TEXTURE_H_

#include <GL/glew.h>

class Texture
{
	public:
		enum TextureType {
			TEXTURE_2D,
			TEXTURE_CUBE
		};

		Texture( GLuint, TextureType );
		virtual ~Texture();

		GLuint getId() const { return this->m_id; };

		bool bind( int textureUnit );

	private:
		static GLint m_currentTextureUnit;

		GLuint m_id;
		TextureType m_textureType;

};

#endif
