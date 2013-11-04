/*
 *  GeometryFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 07.03.13.
 *
 */

#include "GeometryFactory.h"

#include "../ZazenGraphics.h"

#include "../Util/AssImpUtils.h"

#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <iostream>

using namespace std;
using namespace boost;

map<string, MeshNode*> GeometryFactory::allMeshes;
boost::filesystem::path GeometryFactory::modelDataPath;

void
GeometryFactory::setDataPath( const boost::filesystem::path& modelDataPath )
{
	GeometryFactory::modelDataPath = modelDataPath;
}

MeshNode*
GeometryFactory::getMesh( const std::string& fileName )
{
	map<std::string, MeshNode*>::iterator findIter = GeometryFactory::allMeshes.find( fileName );
	if ( findIter != GeometryFactory::allMeshes.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( GeometryFactory::modelDataPath.generic_string() + fileName );
	if ( ! filesystem::exists( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeometryFactory::get: file \"" << fullFileName << "\" does not exist";
		return 0;	
	}

	MeshNode* meshNode = 0;

	if ( filesystem::is_directory( fullFileName ) )
	{
		meshNode = GeometryFactory::loadFolder( fullFileName );
	}
	else
	{
		meshNode = GeometryFactory::loadFile( fullFileName );
	}
	
	if ( NULL != meshNode )
	{
		GeometryFactory::allMeshes[ fileName ] = meshNode;
	}

	return meshNode;
}

MeshStatic*
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
	MeshStatic::StaticVertexData* vertexData = new MeshStatic::StaticVertexData[ numVertices ];
	memset( vertexData, 0, numVertices * sizeof( MeshStatic::StaticVertexData ) );

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

	// put into map => will be cleaned up
	MeshNode* containerNode = new MeshNode( "QUAD_MESH" + GeometryFactory::allMeshes.size() );
	containerNode->m_children.push_back( containerNode );

	MeshStatic* quadMesh = new MeshStatic( numFaces, numVertices, vertexData, indexBuffer );

	GeometryFactory::allMeshes[ containerNode->getName() ] = containerNode;

	return quadMesh;
}

void
GeometryFactory::freeAll()
{
	map<std::string, MeshNode*>::iterator iter = GeometryFactory::allMeshes.begin();
	while ( iter != GeometryFactory::allMeshes.end() )
	{
		delete iter->second;

		iter++;
	}

	GeometryFactory::allMeshes.clear();
}

MeshNode*
GeometryFactory::loadFolder( const filesystem::path& folderPath )
{
	MeshNode* folderGroup = new MeshNode( folderPath.generic_string().c_str() );

	filesystem::directory_iterator iter( folderPath );
	filesystem::directory_iterator endIter;

	// iterate through directory
	while ( iter != endIter )
	{
		filesystem::directory_entry entry = *iter++;
		
		MeshNode* subDirectory = 0;

		if ( filesystem::is_directory( entry.path() ) )
		{
			subDirectory = GeometryFactory::loadFolder( entry.path() );
		}
		else
		{
			subDirectory = GeometryFactory::loadFile( entry.path() );
		}
		
		if ( subDirectory )
		{
			folderGroup->compareAndSetBB( subDirectory->getBBMin(), subDirectory->getBBMin() );
			folderGroup->m_children.push_back( subDirectory );
		}
	}

	return folderGroup;
}

MeshNode*
GeometryFactory::loadFile( const filesystem::path& filePath )
{
	MeshNode* rootNode = NULL;
	glm::vec3 geomGroupBBmin;
	glm::vec3 geomGroupBBmax;

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADING ... " << filePath;

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
		ZazenGraphics::getInstance().getLogger().logError() << "GeometryFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString();
		return 0;
	}

	unsigned int runningBoneIndex = 0;

	rootNode = GeometryFactory::processNode( scene->mRootNode, scene, runningBoneIndex );
	
	aiReleaseImport( scene );

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;

    return rootNode;
}

MeshNode*
GeometryFactory::processNode( const struct aiNode* assImpNode, const struct aiScene* assImpScene, unsigned int& runningBoneIndex )
{
	MeshNode* node = new MeshNode( assImpNode->mName.C_Str() );
	AssImpUtils::assimpMatToGlm( assImpNode->mTransformation, node->m_modelMatrix );

	for ( unsigned int i = 0; i < assImpNode->mNumMeshes; ++i )
	{
		const struct aiMesh* assImpMesh = assImpScene->mMeshes[ assImpNode->mMeshes[ i ] ];

		Mesh* mesh = GeometryFactory::processMesh( assImpMesh, runningBoneIndex );
		if ( NULL != mesh )
		{
			node->compareAndSetBB( mesh->getBBMin(), mesh->getBBMax() );
			node->m_meshes.push_back( mesh );
		}
	}

	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		MeshNode* childNode = GeometryFactory::processNode( assImpNode->mChildren[ i ], assImpScene, runningBoneIndex );
		if ( NULL != childNode )
		{
			node->compareAndSetBB( childNode->getBBMin(), childNode->getBBMax() );
			node->m_children.push_back( childNode );
		}
	}

	return node;
}

Mesh*
GeometryFactory::processMesh( const struct aiMesh* assImpMesh, unsigned int& runningBoneIndex )
{
	// ignore non-triangle meshes
	if ( 0 == ( assImpMesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "ignoring non-triangle mesh " << assImpMesh->mName.C_Str();
		return NULL;
	}

	Mesh* mesh = NULL;

	// this mesh has bones => its supposed to be an animated mesh
	if ( assImpMesh->HasBones() )
	{
		mesh = GeometryFactory::processMeshBoned( assImpMesh, runningBoneIndex );
	}
	else
	{
		mesh = GeometryFactory::processMeshStatic( assImpMesh );
	}

	return mesh;
}

MeshBoned*
GeometryFactory::processMeshBoned( const struct aiMesh* assImpMesh, unsigned int& runningBoneIndex )
{
	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	bool ignoredBones = false;

	GLuint* indexBuffer = NULL;
	MeshBoned::BonedVertexData* vertexData = NULL;

	// allocate vertexdata
	vertexData = new MeshBoned::BonedVertexData[ assImpMesh->mNumVertices ];
	memset( vertexData, 0, assImpMesh->mNumVertices * sizeof( MeshBoned::BonedVertexData ) );

	// indexbuffer for faces
	indexBuffer = new GLuint[ assImpMesh->mNumFaces * 3 ];
	memset( indexBuffer, 0, assImpMesh->mNumFaces * 3 * sizeof( GLuint ) );

	meshBBmin[ 0 ] = numeric_limits<float>::max();
	meshBBmin[ 1 ] = numeric_limits<float>::max();
	meshBBmin[ 2 ] = numeric_limits<float>::max();

	meshBBmax[ 0 ] = numeric_limits<float>::min();
	meshBBmax[ 1 ] = numeric_limits<float>::min();
	meshBBmax[ 2 ] = numeric_limits<float>::min();

	for ( unsigned int i = 0; i < assImpMesh->mNumBones; i++ )
	{
		aiBone* bone = assImpMesh->mBones[ i ];

		// ignore unnamed bones
		if ( 0 == bone->mName.length )
		{
			ZazenGraphics::getInstance().getLogger().logWarning() << "detected unnamed bone - ignoring bone";
			continue;
		}

		//ZazenGraphics::getInstance().getLogger().logWarning() << "bone \"" << bone->mName.C_Str() << "\" has index of " << i + runningBoneIndex;

		for( unsigned int j = 0; j < bone->mNumWeights; j++ )
		{
			aiVertexWeight newWeight = bone->mWeights[ j ];
			MeshBoned::BonedVertexData& vertex = vertexData[ newWeight.mVertexId ];

			// only up to 4 bones per vertex are used
			if ( 4 > vertex.boneCount )
			{
				vertex.boneIndices[ vertex.boneCount ] = i + runningBoneIndex;
				vertex.boneWeights[ vertex.boneCount ] = newWeight.mWeight;
				vertex.boneCount++;
			}
			// already 4 bones have influence on this vertex...
			else
			{
				// search for smallest weight and replace it if the new weight is bigger
				unsigned int minWeightIndex = 0;
				float minWeight = vertex.boneWeights[ 0 ];
	
				for ( unsigned int k = 1; k < vertex.boneCount; k++ )
				{
					if ( vertex.boneWeights[ k ] < minWeight )
					{
						minWeight = vertex.boneWeights[ k ];
						minWeightIndex = k;
					}
				}

				// the new bone has more influence on this vertex then an existing one, replace the existing one
				if ( newWeight.mWeight > minWeight )
				{
					vertex.boneIndices[ minWeightIndex ] = i;
					vertex.boneWeights[ minWeightIndex ] = newWeight.mWeight;
				}

				// for debug-output
				ignoredBones = true;
			}
		}
	}

	// bone-indices must match the bone-indices generated by Animation.cpp which is global for all meshes and not separate
	// => we need for one model global bone-indices and not mesh-specific
	runningBoneIndex += assImpMesh->mNumBones;

	for ( unsigned int i = 0; i < assImpMesh->mNumFaces; ++i )
	{
		const struct aiFace* face = &assImpMesh->mFaces[ i ];

		for( unsigned int j = 0; j < face->mNumIndices; j++ )
		{
			int index = face->mIndices[ j ];

			indexBuffer[ i * 3 + j ] = index;
			memcpy( vertexData[ index ].position, &assImpMesh->mVertices[ index ].x, sizeof( MeshBoned::Vertex ) );
			memcpy( vertexData[ index ].normal, &assImpMesh->mNormals[ index ].x, sizeof( MeshBoned::Normal ) );
			memcpy( vertexData[ index ].tangent, &assImpMesh->mTangents[ index ].x, sizeof( MeshBoned::Tangent ) );

			if ( assImpMesh->HasTextureCoords( 0 ) )
			{
				vertexData[ index ].texCoord[ 0 ] = assImpMesh->mTextureCoords[ 0 ][ index ].x;
				vertexData[ index ].texCoord[ 1 ] = assImpMesh->mTextureCoords[ 0 ][ index ].y;
			}

			GeometryFactory::updateBB( assImpMesh->mVertices[ index ], meshBBmin, meshBBmax );
		}
	}

	if ( ignoredBones )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "ignoring bone-influence on vertex because max bone-per-vertex of 4 exceeded";
	}

	MeshBoned* meshBoned = new MeshBoned( assImpMesh->mNumFaces, assImpMesh->mNumVertices, vertexData, indexBuffer );
	meshBoned->setBB( meshBBmin, meshBBmax );

	return meshBoned;
}

MeshStatic*
GeometryFactory::processMeshStatic( const struct aiMesh* assImpMesh )
{
	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	GLuint* indexBuffer = NULL;
	MeshStatic::StaticVertexData* vertexData = NULL;

	meshBBmin[ 0 ] = numeric_limits<float>::max();
	meshBBmin[ 1 ] = numeric_limits<float>::max();
	meshBBmin[ 2 ] = numeric_limits<float>::max();

	meshBBmax[ 0 ] = numeric_limits<float>::min();
	meshBBmax[ 1 ] = numeric_limits<float>::min();
	meshBBmax[ 2 ] = numeric_limits<float>::min();

	// allocate vertexdata
	vertexData = new MeshStatic::StaticVertexData[ assImpMesh->mNumVertices ];
	memset( vertexData, 0, assImpMesh->mNumVertices * sizeof( MeshStatic::StaticVertexData ) );

	// indexbuffer for faces
	indexBuffer = new GLuint[ assImpMesh->mNumFaces * 3 ];
	memset( indexBuffer, 0, assImpMesh->mNumFaces * 3 * sizeof( GLuint ) );

	for ( unsigned int i = 0; i < assImpMesh->mNumFaces; ++i )
	{
		const struct aiFace* face = &assImpMesh->mFaces[ i ];

		for( unsigned int j = 0; j < face->mNumIndices; j++ )
		{
			int index = face->mIndices[ j ];

			indexBuffer[ i * 3 + j ] = index;
			memcpy( vertexData[ index ].position, &assImpMesh->mVertices[ index ].x, sizeof( MeshStatic::Vertex ) );
			memcpy( vertexData[ index ].normal, &assImpMesh->mNormals[ index ].x, sizeof( MeshStatic::Normal ) );
			memcpy( vertexData[ index ].tangent, &assImpMesh->mTangents[ index ].x, sizeof( MeshStatic::Tangent ) );

			if ( assImpMesh->HasTextureCoords( 0 ) )
			{
				vertexData[ index ].texCoord[ 0 ] = assImpMesh->mTextureCoords[ 0 ][ index ].x;
				vertexData[ index ].texCoord[ 1 ] = assImpMesh->mTextureCoords[ 0 ][ index ].y;
			}

			GeometryFactory::updateBB( assImpMesh->mVertices[ index ], meshBBmin, meshBBmax );
		}
	}

	MeshStatic* meshStatic = new MeshStatic( assImpMesh->mNumFaces, assImpMesh->mNumVertices, vertexData, indexBuffer );
	meshStatic->setBB( meshBBmin, meshBBmax );

	return meshStatic;
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
