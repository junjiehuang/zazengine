/*
 *  model.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 01.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GEOMETRY_FACTORY_LEGACY_H_
#define _GEOMETRY_FACTORY_LEGACY_H_

#include <GL/glew.h>

#include "GeomType.h"

#include <map>
#include <string>

class GeometryFactoryLegacy
{
 	 public:
		 static GeomType* get( const std::string& fileName );
		 static void freeAll();
 	 
 	 private:
		 GeometryFactoryLegacy();
		 ~GeometryFactoryLegacy();

		 static GeomType* load3DS( const std::string& );
		 static GeomType* loadMs3D( const std::string&) ;
		 static GeomType* loadPly( const std::string& ) ;
	
		 static GeomType* loadFolder(const std::string&);

		 static std::map<std::string, GeomType*> meshes;
};

#endif
