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
#include <IL/ilut.h>

#include <boost/filesystem.hpp>

#include <string>
#include <map>

class Texture
{
	public:
		static void init( const boost::filesystem::path& );
		static void freeAll();
	
		static Texture* get( const std::string& );
		static Texture* getCube( const boost::filesystem::path&, const std::string& );

		bool bind( int textureUnit );
	
	private:
		enum TextureType {
			TEXTURE_2D,
			TEXTURE_CUBE
		};

		static GLint m_currentTextureUnit;

		Texture( GLuint, TextureType );
		~Texture();

		GLuint m_textureID;
		TextureType m_textureType;

		static boost::filesystem::path textureDataPath;
		static std::map<std::string, Texture*> allTextures;

		static bool createImages( const std::vector<std::string>&, ILuint** );
		static bool createImage( const std::string&, ILuint* );

		static bool loadImage( const std::string&, ILuint );

		static GLuint createTexture( ILuint );
		static GLuint createCubeTexture( ILuint* );
};

#endif
