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

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

map<string, GeomType*> GeometryFactory::allMeshes;
boost::filesystem::path GeometryFactory::modelDataPath;

void
GeometryFactory::setDataPath( const boost::filesystem::path& modelDataPath )
{
	GeometryFactory::modelDataPath = modelDataPath;
}

GeomType*
GeometryFactory::get( const std::string& fileName )
{
	map<std::string, GeomType*>::iterator findIter = GeometryFactory::allMeshes.find( fileName );
	if ( findIter != GeometryFactory::allMeshes.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( GeometryFactory::modelDataPath.generic_string() + fileName );
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
	{
		GeometryFactory::allMeshes[ fileName ] = geomType;
	}

	return geomType;
}

GeomType*
GeometryFactory::createQuad( float width, float height )
{
	float halfWidth = width / 2;
	float halfHeight = height / 2;

	// consists of 2 triangles
	int numFaces = 2;
	// has 4 vertices
	int numVertices = 4;

	// indexbuffer for faces
	GLuint* indexBuffer = new GLuint[ numFaces * 3 ];
	memset( indexBuffer, 0, numFaces * 3 * sizeof( GLuint ) );

	// allocate vertexdata
	GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ numVertices ];
	memset( vertexData, 0, numVertices * sizeof( GeomMesh::VertexData ) );

	// top left vertex
	vertexData[ 0 ].position[ 0 ] = -halfWidth;
	vertexData[ 0 ].position[ 1 ] = halfHeight;
	vertexData[ 0 ].position[ 2 ] = 0.0f;
	vertexData[ 0 ].normal[ 0 ] = 0.0f;
	vertexData[ 0 ].normal[ 1 ] = 0.0f;
	vertexData[ 0 ].normal[ 2 ] = 1.0f;
	vertexData[ 0 ].texCoord[ 0 ] = 0.0f;
	vertexData[ 0 ].texCoord[ 1 ] = 1.0f;

	// bottom left vertex
	vertexData[ 1 ].position[ 0 ] = -halfWidth;
	vertexData[ 1 ].position[ 1 ] = -halfHeight;
	vertexData[ 1 ].position[ 2 ] = 0.0f;
	vertexData[ 1 ].normal[ 0 ] = 0.0f;
	vertexData[ 1 ].normal[ 1 ] = 0.0f;
	vertexData[ 1 ].normal[ 2 ] = 1.0f;
	vertexData[ 1 ].texCoord[ 0 ] = 0.0f;
	vertexData[ 1 ].texCoord[ 1 ] = 0.0f;

	// bottom right vertex
	vertexData[ 2 ].position[ 0 ] = halfWidth;
	vertexData[ 2 ].position[ 1 ] = -halfHeight;
	vertexData[ 2 ].position[ 2 ] = 0.0f;
	vertexData[ 2 ].normal[ 0 ] = 0.0f;
	vertexData[ 2 ].normal[ 1 ] = 0.0f;
	vertexData[ 2 ].normal[ 2 ] = 1.0f;
	vertexData[ 2 ].texCoord[ 0 ] = 1.0f;
	vertexData[ 2 ].texCoord[ 1 ] = 0.0f;

	// top right vertex
	vertexData[ 3 ].position[ 0 ] = halfWidth;
	vertexData[ 3 ].position[ 1 ] = halfHeight;
	vertexData[ 3 ].position[ 2 ] = 0.0f;
	vertexData[ 3 ].normal[ 0 ] = 0.0f;
	vertexData[ 3 ].normal[ 1 ] = 0.0f;
	vertexData[ 3 ].normal[ 2 ] = 1.0f;
	vertexData[ 3 ].texCoord[ 0 ] = 1.0f;
	vertexData[ 3 ].texCoord[ 1 ] = 1.0f;

	// IMPORTANT: OpenGL defaults front-faces to CounterClockWise => specify indices correct

	// first vertex: top left, bottom left, bottom right
	indexBuffer[ 0 ] = 0;
	indexBuffer[ 1 ] = 1;
	indexBuffer[ 2 ] = 2;

	// second vertex: top left, bottom right, top right
	indexBuffer[ 3 ] = 0;
	indexBuffer[ 4 ] = 2;
	indexBuffer[ 5 ] = 3;

	GeomType* quadMesh = new GeomMesh( numFaces, numVertices, vertexData, indexBuffer );

	// put into map => will be cleaned up
	std::string quadMeshKey = "QUAD_MESH" + GeometryFactory::allMeshes.size();
	GeometryFactory::allMeshes[ quadMeshKey ] = quadMesh;

	return quadMesh;
}

void
GeometryFactory::freeAll()
{
	map<std::string, GeomType*>::iterator iter = GeometryFactory::allMeshes.begin();
	while ( iter != GeometryFactory::allMeshes.end() )
	{
		GeomType* geom = iter->second;
		delete geom;

		iter++;
	}

	GeometryFactory::allMeshes.clear();
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
		
		if ( subFolderGeometryFactory )
		{
			folderGroup->compareBB( subFolderGeometryFactory->getBBMin(), subFolderGeometryFactory->getBBMin() );
			folderGroup->addChild( subFolderGeometryFactory );
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

	const struct aiScene* scene = aiImportFile( fileName.c_str(), aiProcess_CalcTangentSpace	
		| aiProcess_JoinIdenticalVertices 
		| aiProcess_Triangulate 
		| aiProcess_RemoveComponent 
		| aiProcess_GenSmoothNormals 
		| aiProcess_SplitLargeMeshes 
		| aiProcess_ValidateDataStructure 
		| aiProcess_SortByPType 
		| aiProcess_FindInvalidData 
		| aiProcess_GenUVCoords 
		| aiProcess_OptimizeMeshes 
		| aiProcess_OptimizeGraph );
	if ( NULL == scene )
	{
		cout << "ERROR ... in GeometryFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString() << endl;
		return 0;
	}

	geomRoot = GeometryFactory::processNode( scene->mRootNode, scene );
	// can return NULL when contains nothing
	if ( NULL == geomRoot )
	{
		geomRoot->setName( fileName );
	}

	aiReleaseImport( scene );

	cout << "LOADED ... " << filePath << endl;
    
    return geomRoot;
}

GeomType*
GeometryFactory::processNode( const struct aiNode* node, const struct aiScene* scene )
{
	GeomType* geomNode = new GeomType();

	glm::mat4 modelMatrix;
	
	modelMatrix[ 0 ][ 0 ] = node->mTransformation.a1;
	modelMatrix[ 0 ][ 1 ] = node->mTransformation.a2;
	modelMatrix[ 0 ][ 2 ] = node->mTransformation.a3;
	modelMatrix[ 0 ][ 3 ] = node->mTransformation.a4;

	modelMatrix[ 1 ][ 0 ] = node->mTransformation.b1;
	modelMatrix[ 1 ][ 1 ] = node->mTransformation.b2;
	modelMatrix[ 1 ][ 2 ] = node->mTransformation.b3;
	modelMatrix[ 1 ][ 3 ] = node->mTransformation.b4;

	modelMatrix[ 2 ][ 0 ] = node->mTransformation.c1;
	modelMatrix[ 2 ][ 1 ] = node->mTransformation.c2;
	modelMatrix[ 2 ][ 2 ] = node->mTransformation.c3;
	modelMatrix[ 2 ][ 3 ] = node->mTransformation.c4;

	modelMatrix[ 3 ][ 0 ] = node->mTransformation.d1;
	modelMatrix[ 3 ][ 1 ] = node->mTransformation.d2;
	modelMatrix[ 3 ][ 2 ] = node->mTransformation.d3;
	modelMatrix[ 3 ][ 3 ] = node->mTransformation.d4;

	geomNode->setModelMatrix( modelMatrix );

	for ( unsigned int i = 0; i < node->mNumMeshes; ++i )
	{
		const struct aiMesh* mesh = scene->mMeshes[ node->mMeshes[ i ] ];

		GeomType* geomMesh = GeometryFactory::processMesh( mesh );
		if ( NULL != geomMesh )
		{
			geomNode->compareBB( geomMesh->getBBMin(), geomMesh->getBBMax() );
			geomNode->addChild( geomMesh );
		}
	}

	GeometryFactory::processNodeChildren( geomNode, node, scene );

	// ignore empty nodes
	if ( 0 == geomNode->getChildren().size() )
	{
		delete geomNode;
		geomNode = NULL;
	}

	return geomNode;
}

void
GeometryFactory::processNodeChildren( GeomType* geomParent, const struct aiNode* node, const struct aiScene* scene )
{
	for ( unsigned int i = 0; i < node->mNumChildren; i++ )
	{
		GeomType* geomChildNode = GeometryFactory::processNode( node->mChildren[ i ], scene );
		if ( NULL != geomChildNode )
		{
			geomParent->compareBB( geomChildNode->getBBMin(), geomChildNode->getBBMax() );
			geomParent->addChild( geomChildNode );
		}
	}
}

GeomType*
GeometryFactory::processMesh( const struct aiMesh* mesh )
{
	// ignore non-triangle meshes
	if ( 0 == ( mesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE ) )
	{
		return NULL;
	}

	// indexbuffer for faces
	GLuint* indexBuffer = new GLuint[ mesh->mNumFaces * 3 ];
	memset( indexBuffer, 0, mesh->mNumFaces * 3 * sizeof( GLuint ) );

	// allocate vertexdata
	GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ mesh->mNumVertices ];
	memset( vertexData, 0, mesh->mNumVertices * sizeof( GeomMesh::VertexData ) );

	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	meshBBmin[ 0 ] = numeric_limits<float>::max();
	meshBBmin[ 1 ] = numeric_limits<float>::max();
	meshBBmin[ 2 ] = numeric_limits<float>::max();

	meshBBmax[ 0 ] = numeric_limits<float>::min();
	meshBBmax[ 1 ] = numeric_limits<float>::min();
	meshBBmax[ 2 ] = numeric_limits<float>::min();

	for ( unsigned int j = 0; j < mesh->mNumFaces; ++j ) {
		const struct aiFace* face = &mesh->mFaces[ j ];

		for( unsigned int k = 0; k < face->mNumIndices; k++ ) {
			int index = face->mIndices[ k ];

			indexBuffer[ j * 3 + k ] = index;

			memcpy( vertexData[ index ].position, &mesh->mVertices[ index ].x, sizeof( GeomMesh::Vertex ) );
			memcpy( vertexData[ index ].normal, &mesh->mNormals[ index ].x, sizeof( GeomMesh::Normal ) );

			if ( mesh->HasTextureCoords( 0 ) )
			{
				vertexData[ index ].texCoord[ 0 ] = mesh->mTextureCoords[ 0 ][ index ].x;
				vertexData[ index ].texCoord[ 1 ] = mesh->mTextureCoords[ 0 ][ index ].y;
			}

			GeometryFactory::updateBB( mesh->mVertices[ index ], meshBBmin, meshBBmax );
		}
	}

	GeomMesh* geomMesh = new GeomMesh( mesh->mNumFaces, mesh->mNumVertices, vertexData, indexBuffer );
	geomMesh->setBB( meshBBmin, meshBBmax );
	geomMesh->setName( mesh->mName.C_Str() );

	return geomMesh;
}

void
GeometryFactory::updateBB( const aiVector3D& vertex, glm::vec3& meshBBmin, glm::vec3& meshBBmax )
{
	if ( vertex.x < meshBBmin[ 0 ] ) 
	{
		meshBBmin[ 0 ] = vertex.x;
	}
	else if ( vertex.y < meshBBmin[ 1 ] )
	{
		meshBBmin[ 1 ] = vertex.y;
	}
	else if ( vertex.z < meshBBmin[ 2 ] )
	{
		meshBBmin[ 2 ] = vertex.z;
	}

	if ( vertex.x > meshBBmax[ 0 ] ) 
	{
		meshBBmax[ 0 ] = vertex.x;
	}
	else if ( vertex.y > meshBBmax[ 1 ] )
	{
		meshBBmax[ 1 ] = vertex.y;
	}
	else if ( vertex.z > meshBBmax[ 2 ] )
	{
		meshBBmax[ 2 ] = vertex.z;
	}
}