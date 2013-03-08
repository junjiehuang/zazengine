/*
 *  GeometryFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 07.03.13.
 *
 */

#include "GeometryFactory.h"

#include "GeomMesh.h"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>

using namespace std;
using namespace boost;

map<string, GeomType*> GeometryFactory::meshes;

GeomType*
GeometryFactory::get( const std::string& fileName )
{
	map<std::string, GeomType*>::iterator findIter = GeometryFactory::meshes.find( fileName );
	if ( findIter != GeometryFactory::meshes.end() )
		return findIter->second;

	filesystem::path filePath( fileName.c_str() );

	if ( ! filesystem::exists( filePath ) )
	{
		cout << "ERROR ... in GeometryFactory::get: file \"" << fileName << "\" does not exist" << endl;
		return 0;	
	}

	GeomType* geomType = 0;

	if ( filesystem::is_directory( filePath ) )
	{
		geomType = GeometryFactory::loadFolder( filePath );
	}
	else
	{
		geomType = GeometryFactory::loadFile( filePath );
	}
	
	if ( NULL != geomType )
		GeometryFactory::meshes[ fileName ] = geomType;

	return geomType;
}

void
GeometryFactory::freeAll()
{
	map<std::string, GeomType*>::iterator iter = GeometryFactory::meshes.begin();
	while ( iter != GeometryFactory::meshes.end() )
	{
		GeomType* geom = iter->second;
		delete geom;

		iter++;
	}

	GeometryFactory::meshes.clear();
}

GeomType*
GeometryFactory::loadFolder( const filesystem::path& folderPath )
{
	GeomType* folderGroup = new GeomType();

	filesystem::directory_iterator iter( folderPath );
	filesystem::directory_iterator endIter;

	// iterate through directory
	while ( iter != endIter )
	{
		filesystem::directory_entry entry = *iter++;
		
		GeomType* subFolderGeometryFactory = 0;

		if ( filesystem::is_directory( entry.path() ) )
		{
			subFolderGeometryFactory = GeometryFactory::loadFolder( entry.path() );
		}
		else
		{
			subFolderGeometryFactory = GeometryFactory::loadFile( entry.path() );
		}
		
		if ( subFolderGeometryFactory ) {
			subFolderGeometryFactory->parent = folderGroup;
			
			folderGroup->compareBB( subFolderGeometryFactory->getBBMin(), subFolderGeometryFactory->getBBMin() );
			folderGroup->children.push_back( subFolderGeometryFactory );
		}
	}

	return folderGroup;
}

GeomType*
GeometryFactory::loadFile( const filesystem::path& filePath )
{
	GeomType* geom = 0;

	cout << "LOADING ... " << filePath << endl;

	const struct aiScene* scene = aiImportFile( filePath.filename().generic_string().c_str(), aiProcessPreset_TargetRealtime_MaxQuality );
	if ( NULL == scene )
	{
		return 0;
	}

	aiReleaseImport( scene );

	cout << "LOADED ... " << filePath << endl;
    
    return geom;
}
