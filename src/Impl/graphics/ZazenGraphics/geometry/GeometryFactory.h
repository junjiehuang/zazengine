/*
 *  model.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 01.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GEOMETRY_FACTORY_H_
#define _GEOMETRY_FACTORY_H_

#include "GeomType.h"

#include <assimp/scene.h>

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class GeometryFactory
{
public:
	static void init( const boost::filesystem::path& );
	static GeometryFactory& getInstance() { return *GeometryFactory::instance; };

	GeomType* get( const std::string& fileName );
	void freeAll();
 	 
private:
	static GeometryFactory* instance;

	GeometryFactory( const boost::filesystem::path& );
	~GeometryFactory();

	const boost::filesystem::path& m_modelDataPath;

	std::map<std::string, GeomType*> meshes;

	GeomType* loadFolder( const boost::filesystem::path& );
	GeomType* loadFile( const boost::filesystem::path& );

	void processNodeChildren( GeomType* geomParent, const struct aiNode*, const struct aiScene* );
	GeomType* processNode( const struct aiNode*, const struct aiScene* );		 
	GeomType* processMesh( const struct aiMesh* );
};

#endif
