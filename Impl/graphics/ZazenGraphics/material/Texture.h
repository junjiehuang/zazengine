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

#include <string>
#include <map>

class Texture
{
 public:
	static Texture* load( const std::string& );
	static void freeAll();
	
	void bind( int textureUnit );
	void unbind();
	
 private:
	Texture( GLuint );
	~Texture();

	int textureUnit;

	GLuint textureID;
	
	static std::map<std::string, Texture*> allTextures;
};

#endif
