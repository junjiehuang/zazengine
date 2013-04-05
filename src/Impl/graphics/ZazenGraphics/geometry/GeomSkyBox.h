/*
 *  spacebox.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GEOMSPACEBOX_H_
#define _GEOMSPACEBOX_H_

#include "GeomType.h"

#include "../Material/Texture.h"
#include "../Program/Program.h"

class GeomSkyBox : public GeomType
{
	public:
		static bool initialize( const boost::filesystem::path&, const std::string& );
		static bool shutdown();
		static bool isPresent() { return GeomSkyBox::instance != NULL; };

		static GeomSkyBox& getRef() { return *GeomSkyBox::instance;};

		virtual ~GeomSkyBox();
	
		bool render();
	
	private:
		static GeomSkyBox* instance;

		GeomSkyBox();

		Texture* m_frontText;
		GeomType* m_frontGeom;

		Texture* m_backText;
		GeomType* m_backGeom;

		Texture* m_leftText;
		GeomType* m_leftGeom;

		Texture* m_rightText;
		GeomType* m_rightGeom;

		Texture* m_upText;
		GeomType* m_upGeom;

		Texture* m_downText;
		GeomType* m_downGeom;
};

#endif
