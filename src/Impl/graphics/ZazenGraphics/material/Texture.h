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

#include <boost/filesystem.hpp>

#include <string>
#include <map>

class Texture
{
	public:
		static void init( const boost::filesystem::path& );
		static void freeAll();
	
		static Texture* get( const std::string& );

		void bind( int textureUnit );
	
	private:
		Texture( GLuint );
		~Texture();

		GLuint m_textureID;
	
		static boost::filesystem::path textureDataPath;
		static std::map<std::string, Texture*> allTextures;

		static GLuint createGLTexture( const boost::filesystem::path& );
};

#endif
