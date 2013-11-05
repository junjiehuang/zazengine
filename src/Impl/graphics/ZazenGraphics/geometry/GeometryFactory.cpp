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

GeometryFactory* GeometryFactory::instance = NULL;

void
GeometryFactory::init( const boost::filesystem::path& modelDataPath )
{
	if ( NULL == GeometryFactory::instance )
	{
		new GeometryFactory( modelDataPath );
	}
}

void
GeometryFactory::freeAll()
{
	if ( GeometryFactory::instance )
	{
		delete GeometryFactory::instance;
	}
}

GeometryFactory::GeometryFactory( const boost::filesystem::path& modelDataPath )
	: m_modelDataPath( modelDataPath )
{
	GeometryFactory::instance = this;
}

GeometryFactory::~GeometryFactory()
{
	map<std::string, MeshNode*>::iterator iter = this->m_allMeshes.begin();
	while ( iter != this->m_allMeshes.end() )
	{
		delete iter->second;

		iter++;
	}

	GeometryFactory::instance = NULL;
}

MeshNode*
GeometryFactory::getMesh( const std::string& fileName )
{
	map<std::string, MeshNode*>::iterator findIter = this->m_allMeshes.find( fileName );
	if ( findIter != this->m_allMeshes.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( this->m_modelDataPath.generic_string() + fileName );
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
		this->m_allMeshes[ fileName ] = meshNode;
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
	MeshNode* containerNode = new MeshNode( "QUAD_MESH" + this->m_allMeshes.size() );
	MeshStatic* quadMesh = new MeshStatic( numFaces, numVertices, vertexData, indexBuffer );

	containerNode->m_meshes.push_back( quadMesh );

	this->m_allMeshes[ containerNode->getName() ] = containerNode;

	return quadMesh;
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

	this->m_currentScene = aiImportFile( fileName.c_str(), aiProcess_CalcTangentSpace	
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
	if ( NULL == this->m_currentScene )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GeometryFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString();
		return 0;
	}
	
	// need to collect all bones hierarchical to construct correct bone-indices
	// animation will build then an animation-skeleton with the hierarchy information
	// where the bone-indices will be constructed the same way => indices must mach
	GeometryFactory::collectBonesHierarchical( this->m_currentScene->mRootNode );

	// walk and create hieararchy recursive
	rootNode = GeometryFactory::processNode( this->m_currentScene->mRootNode );

	aiReleaseImport( this->m_currentScene );
	this->m_currentScene = NULL;
	this->m_currentBonesHierarchical.clear();

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;

    return rootNode;
}

MeshNode*
GeometryFactory::processNode( const struct aiNode* assImpNode )
{
	MeshNode* node = new MeshNode( assImpNode->mName.C_Str() );
	AssImpUtils::assimpMatToGlm( assImpNode->mTransformation, node->m_modelMatrix );

	// find matching bone to this hierarchy-node, is optional!
	int boneIndex = this->getBoneIndex( node->m_name );
	if ( -1 != boneIndex )
	{
		node->m_bone = new MeshNode::MeshBone();
		*node->m_bone = this->m_currentBonesHierarchical[ boneIndex ];
	}
	
	// collect all meshes of this node
	for ( unsigned int i = 0; i < assImpNode->mNumMeshes; ++i )
	{
		const struct aiMesh* assImpMesh = this->m_currentScene->mMeshes[ assImpNode->mMeshes[ i ] ];

		Mesh* mesh = this->processMesh( assImpMesh );
		if ( NULL != mesh )
		{
			node->compareAndSetBB( mesh->getBBMin(), mesh->getBBMax() );
			node->m_meshes.push_back( mesh );
		}
	}

	// walk tree down recursive
	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		MeshNode* childNode = this->processNode( assImpNode->mChildren[ i ] );
		node->compareAndSetBB( childNode->getBBMin(), childNode->getBBMax() );
		node->m_children.push_back( childNode );
	}

	return node;
}

Mesh*
GeometryFactory::processMesh( const struct aiMesh* assImpMesh )
{
	// ignore non-triangle meshes
	if ( 0 == ( assImpMesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE ) )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "ignoring non-triangle mesh " << assImpMesh->mName.C_Str();
		return NULL;
	}

	Mesh* mesh = NULL;

	// this mesh has bones => its supposed to be an animated mesh
	if ( assImpMesh->HasBones() )
	{
		mesh = GeometryFactory::processMeshBoned( assImpMesh );
	}
	else
	{
		mesh = GeometryFactory::processMeshStatic( assImpMesh );
	}

	return mesh;
}

MeshBoned*
GeometryFactory::processMeshBoned( const struct aiMesh* assImpMesh )
{
	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

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

	// iterate over all affecting bones to collect weights
	for ( unsigned int i = 0; i < assImpMesh->mNumBones; i++ )
	{
		aiBone* bone = assImpMesh->mBones[ i ];

		// ignore unnamed bones
		if ( 0 == bone->mName.length )
		{
			ZazenGraphics::getInstance().getLogger().logWarning() << "detected unnamed bone - ignoring bone";
			continue;
		}

		// get index of bone in hierarchy: should never be negative because for each bone there will be an according node
		// (note: the other way round does not hold true: for each node there is NOT ALWAYS an according bone)
		unsigned int boneIndex = this->getBoneIndex( bone->mName.C_Str() );
		
		for( unsigned int j = 0; j < bone->mNumWeights; j++ )
		{
			aiVertexWeight& newWeight = bone->mWeights[ j ];
			MeshBoned::BonedVertexData& vertex = vertexData[ newWeight.mVertexId ];

			GeometryFactory::processBoneWeight( vertex, newWeight, boneIndex );
		}
	}

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

	MeshBoned* meshBoned = new MeshBoned( assImpMesh->mNumFaces, assImpMesh->mNumVertices, vertexData, indexBuffer );
	meshBoned->setBB( meshBBmin, meshBBmax );

	return meshBoned;
}

void
GeometryFactory::processBoneWeight( MeshBoned::BonedVertexData& vertex, const aiVertexWeight& newWeight, unsigned int boneIndex )
{
	// only up to 4 bones per vertex are used
	if ( 4 > vertex.boneCount )
	{
		vertex.boneIndices[ vertex.boneCount ] = boneIndex;
		vertex.boneWeights[ vertex.boneCount ] = newWeight.mWeight;
		vertex.boneCount++;
	}
	// already 4 bones have influence on this vertex...
	else
	{
		// search for smallest weight and replace it if the new weight is bigger
		unsigned int minWeightIndex = 0;
		float minWeight = vertex.boneWeights[ 0 ];
	
		for ( unsigned int i = 1; i < vertex.boneCount; i++ )
		{
			if ( vertex.boneWeights[ i ] < minWeight )
			{
				minWeight = vertex.boneWeights[ i ];
				minWeightIndex = i;
			}
		}

		// the new bone has more influence on this vertex then an existing one, replace the existing one
		if ( newWeight.mWeight > minWeight )
		{
			vertex.boneIndices[ minWeightIndex ] = boneIndex;
			vertex.boneWeights[ minWeightIndex ] = newWeight.mWeight;
		}
	}
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
GeometryFactory::collectBonesHierarchical( const struct aiNode* assImpNode )
{
	for ( unsigned int i = 0; i < this->m_currentScene->mNumMeshes; i++ )
	{
		aiMesh* assImpMesh = this->m_currentScene->mMeshes[ i ];

		for ( unsigned int j = 0; j < assImpMesh->mNumBones; j++ )
		{
			aiBone* assImpBone = assImpMesh->mBones[ j ];
			string assImpBoneName =  assImpBone->mName.C_Str();
			if ( assImpBoneName == assImpNode->mName.C_Str() )
			{
				MeshNode::MeshBone bone;
				bone.m_name = assImpBone->mName.C_Str();
				AssImpUtils::assimpMatToGlm( assImpBone->mOffsetMatrix, bone.m_offset );

				this->m_currentBonesHierarchical.push_back( bone );
			}
		}
	}

	// walk the tree recursive
	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		this->collectBonesHierarchical( assImpNode->mChildren[ i ] );
	}
}

int
GeometryFactory::getBoneIndex( const std::string& boneName )
{
	for ( unsigned int i = 0; i < this->m_currentBonesHierarchical.size(); i++ )
	{
		if ( boneName == this->m_currentBonesHierarchical[ i ].m_name )
		{
			return i;
		}
	}

	return -1;
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
