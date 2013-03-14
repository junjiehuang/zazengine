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
#include <assimp/postprocess.h>

#include <iostream>

using namespace std;
using namespace boost;

GeometryFactory* GeometryFactory::instance = NULL;

void
GeometryFactory::init( const boost::filesystem::path& modelDataPath )
{
	if ( NULL == GeometryFactory::instance )
	{
		new GeometryFactory( modelDataPath );
	}
}

GeometryFactory::GeometryFactory( const boost::filesystem::path& modelDataPath )
	: m_modelDataPath( modelDataPath )
{
	GeometryFactory::instance = this;
}

GeometryFactory::~GeometryFactory()
{
	GeometryFactory::instance = NULL;
}

GeomType*
GeometryFactory::get( const std::string& fileName )
{
	map<std::string, GeomType*>::iterator findIter = GeometryFactory::meshes.find( fileName );
	if ( findIter != GeometryFactory::meshes.end() )
		return findIter->second;

	filesystem::path fullFileName( this->m_modelDataPath.generic_string() + fileName );

	if ( ! filesystem::exists( fullFileName ) )
	{
		cout << "ERROR ... in GeometryFactory::get: file \"" << fullFileName << "\" does not exist" << endl;
		return 0;	
	}

	GeomType* geomType = 0;

	if ( filesystem::is_directory( fullFileName ) )
	{
		geomType = GeometryFactory::loadFolder( fullFileName );
	}
	else
	{
		geomType = GeometryFactory::loadFile( fullFileName );
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
	GeomType* geomRoot = 0;
	glm::vec3 geomGroupBBmin;
	glm::vec3 geomGroupBBmax;

	cout << "LOADING ... " << filePath << endl;

	const std::string& fileName = filePath.generic_string();

	const struct aiScene* scene = aiImportFile( fileName.c_str(), aiProcessPreset_TargetRealtime_MaxQuality );
	if ( NULL == scene )
	{
		return 0;
	}
	
	geomRoot = new GeomType();
	geomRoot->name = fileName;

	const struct aiNode* rootNode = scene->mRootNode;

	GeometryFactory::processNodeChildren( geomRoot, rootNode, scene );
	
	aiReleaseImport( scene );

	cout << "LOADED ... " << filePath << endl;
    
    return geomRoot;
}

void
GeometryFactory::processNodeChildren( GeomType* geomParent, const struct aiNode* node, const struct aiScene* scene )
{
	for ( unsigned int i = 0; i < node->mNumChildren; i++ )
	{
		GeomType* geomChildNode = GeometryFactory::processNode( node->mChildren[ i ], scene );
		if ( NULL != geomChildNode )
		{
			geomParent->children.push_back( geomChildNode );
		}
	}
}

GeomType*
GeometryFactory::processNode( const struct aiNode* node, const struct aiScene* scene )
{
	GeomType* geomParent = new GeomType();

	for ( unsigned int i = 0; i < node->mNumMeshes; ++i )
	{
		const struct aiMesh* mesh = scene->mMeshes[ node->mMeshes[ i ] ];

		GeomType* geomMesh = GeometryFactory::processMesh( mesh );
		geomParent->children.push_back( geomMesh );
	}

	GeometryFactory::processNodeChildren( geomParent, node, scene );

	return geomParent;
}

GeomType*
GeometryFactory::processMesh( const struct aiMesh* mesh )
{
	// TODO take transformation of nodes into account

	// indexbuffer for faces
	GLuint* indexBuffer = new GLuint[ mesh->mNumFaces * 3 ];
	memset( indexBuffer, 0, mesh->mNumFaces * 3 * sizeof( GLuint ) );

	// allocate vertexdata
	GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ mesh->mNumVertices ];
	memset( vertexData, 0, mesh->mNumVertices * sizeof( GeomMesh::VertexData ) );

	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	// TODO calculate bounding-box

	for ( unsigned int j = 0; j < mesh->mNumFaces; ++j ) {
		const struct aiFace* face = &mesh->mFaces[ j ];

		for( unsigned int k = 0; k < face->mNumIndices; k++ ) {
			int index = face->mIndices[ k ];

			indexBuffer[ j * 3 + k ] = index;

			memcpy( vertexData[ index ].position, &mesh->mVertices[ index ].x, sizeof( GeomMesh::Vertex ) );
			memcpy( vertexData[ index ].normal, &mesh->mNormals[ index ].x, sizeof( GeomMesh::Normal ) );

			vertexData[ index ].texCoord[ 0 ] = mesh->mTextureCoords[ 0 ][ index].x;
			vertexData[ index ].texCoord[ 1 ] = mesh->mTextureCoords[ 0 ][ index].y;
		}
	}

	GeomMesh* geomMesh = new GeomMesh( mesh->mNumFaces, mesh->mNumVertices, vertexData, indexBuffer );
	geomMesh->setBB( meshBBmin, meshBBmax );
	geomMesh->name = mesh->mName.C_Str();

	return geomMesh;
}