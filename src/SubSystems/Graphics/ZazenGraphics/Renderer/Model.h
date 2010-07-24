/*
 *  model.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 01.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _MODEL_H_
#define _MODEL_H_

#include <GL/glew.h>

#include "geom/GeomType.h"

#include <map>
#include <string>

class Model
{
 public:
	 static GeomType* get(const std::string&);
	 static void loadMesh(const std::string&, const std::string&);
	 static void registerGeom(GeomType* t, const std::string& id) { Model::models[id] = t; };

	 static void freeAll();
 	 
 private:
	Model();
	~Model();

	static GeomType* load3DS(const std::string&);
	static GeomType* loadMs3D(const std::string&);
	static GeomType* loadPly(const std::string&);

	static GeomType* loadFolder(const std::string&);
	
	static std::map<std::string, GeomType*> models;
	
};

#endif
