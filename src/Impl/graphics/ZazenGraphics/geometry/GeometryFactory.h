/*
 *  GeometryFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 07.03.13.
 *
 */

#ifndef _GEOMETRY_FACTORY_H_
#define _GEOMETRY_FACTORY_H_

#include "MeshStatic.h"
#include "MeshBoned.h"

#include <assimp/scene.h>

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class GeometryFactory
{
	public:
		static void setDataPath( const boost::filesystem::path& );
		static void freeAll();
		static void free( Mesh* );

		static Mesh* getMesh( const std::string& fileName );

		static MeshStatic* createQuad( float width, float height );

	private:
		static boost::filesystem::path modelDataPath;
		static std::map<std::string, Mesh*> allMeshes;

		static Mesh* loadFolder( const boost::filesystem::path& );
		static Mesh* loadFile( const boost::filesystem::path& );

		static Mesh* processNode( const struct aiNode*, const struct aiScene*, unsigned int& );		 
		static Mesh* processMesh( const struct aiMesh*, unsigned int& );

		static MeshBoned* processMeshBoned( const struct aiMesh*, unsigned int& );
		static MeshStatic* processMeshStatic( const struct aiMesh* );

		static void updateBB( const aiVector3D& vertex, glm::vec3&, glm::vec3& );

};

#endif
