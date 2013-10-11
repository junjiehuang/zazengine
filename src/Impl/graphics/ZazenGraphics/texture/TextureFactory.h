/*
 *  TextureFactory.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 30.04.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _TEXTUREFACTORY_H_
#define _TEXTUREFACTORY_H_

#include "Texture.h"

#include <GL/glew.h>
#include <IL/ilut.h>

#include <boost/filesystem.hpp>

#include <string>
#include <map>

class TextureFactory
{
	public:
		static void init( const boost::filesystem::path& );
		static void freeAll();
	
		static Texture* get( const std::string& );
		static Texture* getCube( const boost::filesystem::path&, const std::string& );

		bool bind( int textureUnit );
	
	private:
		static GLint m_currentTextureUnit;

		static boost::filesystem::path textureDataPath;
		static std::map<std::string, Texture*> allTextures;

		static GLuint createTexture( const std::string& fullFileName );
		static GLuint createCubeTexture( const std::vector<std::string>& fileNames );

		static bool createImages( const std::vector<std::string>&, ILuint* );
		static bool createImage( const std::string&, ILuint* );

		static bool loadImage( const std::string&, ILuint );

};

#endif
