/*
 *  GeometryFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 07.03.13.
 *
 */

#include "GeometryFactory.h"

#include "GeomMesh.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

#include <glm/gtc/type_ptr.hpp>

using namespace std;
using namespace boost;

map<string, GeomType*> GeometryFactory::meshes;

GeomType*
GeometryFactory::get( const std::string& fileName )
{
	map<std::string, GeomType*>::iterator findIter = GeometryFactory::meshes.find( fileName );
	if ( findIter != GeometryFactory::meshes.end() )
		return findIter->second;

	string ending;
	unsigned long index = fileName.find_last_of('.');
	if (index != string::npos)
		ending = fileName.substr(index + 1, fileName.length() - index);

	GeomType* geomType = 0;

	if ( boost::iequals( ending.c_str(), "ply" ) )
	{
		geomType = GeometryFactory::loadPly(fileName);
	}
	else if ( boost::iequals( ending.c_str(), "ms3d" ) )
	{
		geomType = GeometryFactory::loadMs3D(fileName);
	}
	else if ( boost::iequals( ending.c_str(), "3ds" ) )
	{
		geomType = GeometryFactory::load3DS(fileName);
	}
	else
	{
		geomType = GeometryFactory::loadFolder(fileName);
	}

	if ( geomType )
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
GeometryFactory::loadFolder( const std::string& folderName )
{
	string fullPath = folderName;
	filesystem::path directory( fullPath.c_str() );

	if ( ! filesystem::exists( directory ) )
	{
			cout << "ERROR ... couldn't open directory \"" << fullPath << "\" in GeometryFactory::loadFolder" << endl;
			return 0;	
	}

	if ( ! filesystem::is_directory( directory ) )
	{
			cout << "ERROR ... in GeometryFactory::loadFolder: expected directory at \"" << fullPath << "\" but is no directory" << endl;
			return 0;	
	}
	
	GeomType* folderGroup = new GeomType();

	filesystem::directory_iterator iter( directory );
	filesystem::directory_iterator endIter();

	// iterate through directory
	while ( iter != endIter() )
	{
		filesystem::directory_entry entry = *iter++;

		string fileName = entry.path().filename().generic_string();
		string fullFileName = fullPath + "/" + fileName;
		
		GeomType* subFolderGeometryFactory = 0;
		string subFolderPath = folderName + "/" + fileName;

		if ( filesystem::is_directory( entry.path() ) )
		{
			//cout << "subFolderPath = " << subFolderPath << endl;
			subFolderGeometryFactory = GeometryFactory::loadFolder( subFolderPath );
		}
		else
		{
			string ending = entry.path().extension().generic_string();
			unsigned long index = fullFileName.find_last_of('.');
			if (index != string::npos)
				ending = fullFileName.substr(index + 1, fullFileName.length() - index);

			//cout << "subFilePath = " << fullFileName << endl;

			if ( boost::iequals( ending.c_str(), ".ply" ) )
			{
				subFolderGeometryFactory = GeometryFactory::loadPly(fullFileName);
			} 
			else if ( boost::iequals( ending.c_str(), ".ms3d" ) )
			{
				subFolderGeometryFactory = GeometryFactory::loadMs3D(fullFileName);
			}
			else if ( boost::iequals( ending.c_str(), ".3ds" ) )
			{
				subFolderGeometryFactory = GeometryFactory::load3DS(fullFileName);
			} 
			else {
				cout << "bad ending: \"" << ending << "\"" << endl;
			}
		}
		
		if ( subFolderGeometryFactory ) {
			subFolderGeometryFactory->parent = folderGroup;
			
			folderGroup->compareBB(subFolderGeometryFactory->getBBMin(), subFolderGeometryFactory->getBBMin());
			folderGroup->children.push_back(subFolderGeometryFactory);
		}
	}

	return folderGroup;
}

GeomType*
GeometryFactory::load3DS(const std::string& fileName)
{
	GeomType* geom = 0;

    cout << "LOADING ... " << fileName << endl;

	cout << "LOADED ... " << fileName << endl;
    
    return geom;
}

GeomType*
GeometryFactory::loadMs3D(const std::string& fileName)
{
	GeomType* geom = 0;
	
	cout << "LOADING ... " << fileName << endl;

	cout << "LOADED ... " << fileName << endl;

	return geom;
}

GeomType*
GeometryFactory::loadPly(const std::string& fileName)
{	
	GeomType* geom = 0;

	cout << "LOADING ... " << fileName << endl;

	cout << "LOADED ... " << fileName << endl;

	return geom;
}
