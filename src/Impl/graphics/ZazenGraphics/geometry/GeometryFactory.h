/*
 *  GeometryFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 07.03.13.
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
		static void setDataPath( const boost::filesystem::path& );
		static void freeAll();
		static void free( GeomType* );

		static GeomType* get( const std::string& fileName );

		static GeomType* createQuad( float width, float height );

	private:
		static boost::filesystem::path modelDataPath;
		static std::map<std::string, GeomType*> allMeshes;

		static GeomType* loadFolder( const boost::filesystem::path& );
		static GeomType* loadFile( const boost::filesystem::path& );

		static void processNodeChildren( GeomType* geomParent, const struct aiNode*, const struct aiScene* );
		static GeomType* processNode( const struct aiNode*, const struct aiScene* );		 
		static GeomType* processMesh( const struct aiMesh* );

		static GeomType* processAnimatedMesh( const struct aiMesh* );
		static GeomType* processStaticMesh( const struct aiMesh* );

		static void updateBB( const aiVector3D& vertex, glm::vec3&, glm::vec3& );

};

#endif
