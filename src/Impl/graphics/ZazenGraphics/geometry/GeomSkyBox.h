/*
 *  spacebox.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GEOMSKYBOX_H_
#define _GEOMSKYBOX_H_

#include "GeomType.h"

#include "../Texture/Texture.h"
#include "../Program/Program.h"
#include "../Program/UniformBlock.h"

#include <boost/filesystem.hpp>

class GeomSkyBox : public GeomType
{
	public:
		static bool initialize( const boost::filesystem::path&, const std::string& );
		static bool shutdown();
		static bool isPresent() { return GeomSkyBox::instance != NULL; };

		static GeomSkyBox& getRef() { return *GeomSkyBox::instance;};

		virtual ~GeomSkyBox();
	
		bool render();
	
		void setTransformBlock( UniformBlock* block ) { this->m_transformsBlock = block; };

	private:
		static GeomSkyBox* instance;

		GLuint m_dataVBO;
		GLuint m_indexVBO;
		Texture* m_cubeMap;
		UniformBlock* m_transformsBlock;

		GeomSkyBox();
};

#endif
