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
#include <iostream>

#include <glm/gtc/type_ptr.hpp>

using namespace std;

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
GeometryFactory::loadFolder(const std::string& folderName)
{
	// TODO use boost filesystem stuff for this instead of system-api

	string fullPath = folderName;
	DIR* directory = opendir(fullPath.c_str());
	if (!directory)
	{
			cout << "ERROR ... couldn't open Directory \"" << fullPath << "\" in GeometryFactory::loadFolder" << endl;
			return 0;	
	}
	
	GeomType* folderGroup = new GeomType();
	
	struct dirent* entry;
	struct stat entryStatus;

	// iterate through directory
	while ((entry = readdir(directory)) != NULL)
	{
		string fileName = entry->d_name;
		string fullFileName = fullPath + "/" + fileName;
		
		stat(fullFileName.c_str(), &entryStatus);

		// ignore .
		if(fileName == ".")
			continue;
		
		// ignore
		if (fileName == "..")
			continue;

		GeomType* subFolderGeometryFactory = 0;
		string subFolderPath = folderName + "/" + fileName;

		if (S_ISDIR(entryStatus.st_mode))
		{
			//cout << "subFolderPath = " << subFolderPath << endl;
			subFolderGeometryFactory = GeometryFactory::loadFolder(subFolderPath);

		}
		else
		{
			string ending;
			unsigned long index = fullFileName.find_last_of('.');
			if (index != string::npos)
				ending = fullFileName.substr(index + 1, fullFileName.length() - index);

			//cout << "subFilePath = " << fullFileName << endl;

			if (strcasecmp(ending.c_str(), "ply") == 0) {
				subFolderGeometryFactory = GeometryFactory::loadPly(fullFileName);
			} else if (strcasecmp(ending.c_str(), "ms3d") == 0) {
				subFolderGeometryFactory = GeometryFactory::loadMs3D(fullFileName);
			} else if (strcasecmp(ending.c_str(), "3ds") == 0) {
				subFolderGeometryFactory = GeometryFactory::load3DS(fullFileName);
			} else {
				cout << "bad ending: \"" << ending << "\"" << endl;
			}
		}
		
		if (subFolderGeometryFactory) {
			subFolderGeometryFactory->parent = folderGroup;
			
			folderGroup->compareBB(subFolderGeometryFactory->getBBMin(), subFolderGeometryFactory->getBBMin());
			folderGroup->children.push_back(subFolderGeometryFactory);
		}
	}

	closedir(directory);

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
