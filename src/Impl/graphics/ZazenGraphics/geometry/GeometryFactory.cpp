#include "GeometryFactory.h"

#include "../ZazenGraphics.h"

#include "../Util/AssImpUtils.h"
#include "../Util/GLUtils.h"

#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <iostream>
#include <cmath>

using namespace std;
using namespace boost;

#define M_PI       3.14159265f
#define M_PI_2     1.57079632f

#define BUFFER_OFFSET( i ) ( ( char* ) NULL + ( i ) )

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

	this->m_currentScene = NULL;

	this->m_ndcCube = NULL;
	this->m_ndcQuad = NULL;
	this->m_unitSphere = NULL;
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

Mesh*
GeometryFactory::getNDCQuad()
{
	// already instantiated, hand out the ptr to it
	if ( this->m_ndcQuad )
	{
		return this->m_ndcQuad;
	}

	// not yet instantiated, lazy load...
	
	// TODO: remove tex-coords when finished with transparency-stage

	// indexbuffer for faces
	GLuint indexData[ 6 ];
	memset( indexData, 0, 6 * sizeof( GLuint ) );

	// allocate vertexdata
	QuadVertexData vertexData[ 4 ];
	memset( vertexData, 0, 4 * sizeof( QuadVertexData ) );

	// top left vertex
	vertexData[ 0 ].position[ 0 ] = -1.0;
	vertexData[ 0 ].position[ 1 ] = 1.0;
	vertexData[ 0 ].position[ 2 ] = 0.0f;
	vertexData[ 0 ].texCoord[ 0 ] = 0.0f;
	vertexData[ 0 ].texCoord[ 1 ] = 1.0f;

	// bottom left vertex
	vertexData[ 1 ].position[ 0 ] = -1.0;
	vertexData[ 1 ].position[ 1 ] = -1.0;
	vertexData[ 1 ].position[ 2 ] = 0.0f;
	vertexData[ 1 ].texCoord[ 0 ] = 0.0f;
	vertexData[ 1 ].texCoord[ 1 ] = 0.0f;

	// bottom right vertex
	vertexData[ 2 ].position[ 0 ] = 1.0;
	vertexData[ 2 ].position[ 1 ] = -1.0;
	vertexData[ 2 ].position[ 2 ] = 0.0f;
	vertexData[ 2 ].texCoord[ 0 ] = 1.0f;
	vertexData[ 2 ].texCoord[ 1 ] = 0.0f;

	// top right vertex
	vertexData[ 3 ].position[ 0 ] = 1.0;
	vertexData[ 3 ].position[ 1 ] = 1.0;
	vertexData[ 3 ].position[ 2 ] = 0.0f;
	vertexData[ 3 ].texCoord[ 0 ] = 1.0f;
	vertexData[ 3 ].texCoord[ 1 ] = 1.0f;

	// IMPORTANT: OpenGL defaults front-faces to CounterClockWise => specify indices correct
	// first vertex: top left, bottom left, bottom right
	indexData[ 0 ] = 0;
	indexData[ 1 ] = 1;
	indexData[ 2 ] = 2;

	// second vertex: top left, bottom right, top right
	indexData[ 3 ] = 0;
	indexData[ 4 ] = 2;
	indexData[ 5 ] = 3;

	GLuint vao = 0;
	GLuint dataVBO = 0;
	GLuint indexVBO = 0;

	// NOTE: the VAO encapuslates ALL of the subsequent buffers and states of the buffers 
	glGenVertexArrays( 1, &vao ); 
	glBindVertexArray( vao );

	// generate and setup data vbo
	glGenBuffers( 1, &dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ARRAY_BUFFER, dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ARRAY_BUFFER, sizeof( QuadVertexData ) * 4, vertexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to coordinates
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( QuadVertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to texture coords
	glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( QuadVertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG
	
	// generate and setup index vbo
	glGenBuffers( 1, &indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * 6, indexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindVertexArray( 0 );

	this->m_ndcQuad = new Mesh( vao, dataVBO, indexVBO, 6 );
	this->m_ndcQuad->m_vertexData = NULL;	// no dynamic memory for small mesh 
	this->m_ndcQuad->m_indexData = NULL;	// no dynamic memory for small mesh 

	// put into map => will be cleaned up
	MeshNode* containerNode = new MeshNode( "NDC_QUAD" );
	containerNode->m_meshes.push_back( this->m_ndcQuad );

	this->m_allMeshes[ containerNode->getName() ] = containerNode;

	return this->m_ndcQuad;
}

Mesh*
GeometryFactory::getNDCCube()
{
	// already instantiated, hand out the ptr to it
	if ( this->m_ndcCube )
	{
		return this->m_ndcCube;
	}

	// not yet instantiated, lazy load...
	
	GLuint vao = 0;
	GLuint dataVBO = 0;
	GLuint indexVBO = 0;

	// cube vertices for vertex buffer object
	GLfloat cube_vertices[] = {
		-1.0,  1.0,  1.0,
		-1.0, -1.0,  1.0,
		1.0, -1.0,  1.0,
		1.0,  1.0,  1.0,
		-1.0,  1.0, -1.0,
		-1.0, -1.0, -1.0,
		1.0, -1.0, -1.0,
		1.0,  1.0, -1.0,
	};

	// cube indices for index buffer object
	// NOTE: specify indices in CCW (is the default for OpenGL and not changed in this renderer)
	GLuint cube_indices[] = {
		// pos z
		0, 1, 2,
		2, 3, 0,
		// neg z
		7, 6, 5,
		5, 4, 7,
		// pos y
		4, 0, 3,
		3, 7, 4,
		// neg Y
		5, 1, 2,
		2, 6, 5,
		// pos x
		4, 5, 1,
		1, 0, 4,
		// neg x
		3, 2, 6,
		6, 7, 3
	};

	// NOTE: the VAO encapuslates ALL of the subsequent buffers and states of the buffers 
	glGenVertexArrays( 1, &vao ); 
	glBindVertexArray( vao );

	glGenBuffers( 1, &dataVBO );
	GL_PEEK_ERRORS_AT

	glBindBuffer( GL_ARRAY_BUFFER, dataVBO );
	GL_PEEK_ERRORS_AT

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ARRAY_BUFFER, sizeof( cube_vertices ), cube_vertices, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT

	glGenBuffers( 1, &indexVBO );
	GL_PEEK_ERRORS_AT
	
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexVBO );
	GL_PEEK_ERRORS_AT
	
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( cube_indices ), cube_indices, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT
	
	glBindVertexArray( 0 );

	this->m_ndcCube = new Mesh( vao, dataVBO, indexVBO, 36 );
	this->m_ndcCube->m_vertexData = NULL;	// no dynamic memory for small mesh 
	this->m_ndcCube->m_indexData = NULL;	// no dynamic memory for small mesh 

	// put into map => will be cleaned up
	MeshNode* containerNode = new MeshNode( "NDC_CUBE" );
	containerNode->m_meshes.push_back( this->m_ndcCube );

	this->m_allMeshes[ containerNode->getName() ] = containerNode;

	return this->m_ndcCube;
}

Mesh*
GeometryFactory::getUnitSphere()
{
	// already instantiated, hand out the ptr to it
	if ( this->m_unitSphere )
	{
		return this->m_unitSphere;
	}

	GLuint vao = 0;
	GLuint dataVBO = 0;
	GLuint indexVBO = 0;

	unsigned int vertexIndex = 0;
	unsigned int indexIndex = 0;

	float radius = 1.0;
	unsigned int rings = 20;
	unsigned int sectors = 20;

	// code from http://stackoverflow.com/questions/5988686/creating-a-3d-sphere-in-opengl-using-visual-c/5989676#5989676
	std::vector<GLfloat> sphereVertices;
    std::vector<GLuint> sphereIndices;

	sphereVertices.resize( rings * sectors * 3 );
	sphereIndices.resize( rings * sectors * 3 * 2 );

	float const R = 1.0f / ( float ) ( rings - 1 );
    float const S = 1.0f / ( float ) ( sectors - 1 );

    for( unsigned int r = 0; r < rings; ++r )
	{
        for( unsigned int s = 0; s < sectors; ++s )
		{
            float const y = sin( -M_PI_2 + M_PI * r * R );
            float const x = cos( 2 * M_PI * s * S ) * sin( M_PI * r * R );
            float const z = sin( 2 * M_PI * s * S ) * sin( M_PI * r * R );
			int curRow = r * sectors;
			int nextRow = ( r + 1 ) * sectors;

            sphereVertices[ vertexIndex++ ] = x * radius;
			sphereVertices[ vertexIndex++ ] = y * radius;
			sphereVertices[ vertexIndex++ ] = z * radius;

			sphereIndices[ indexIndex++ ] = curRow + s;
			sphereIndices[ indexIndex++ ] = nextRow + s;
			sphereIndices[ indexIndex++ ] = nextRow + ( s + 1 );

			sphereIndices[ indexIndex++ ] = curRow + s;
			sphereIndices[ indexIndex++ ] = nextRow + ( s + 1 );
			sphereIndices[ indexIndex++ ] = curRow + ( s + 1 );
        }
    }

	// NOTE: the VAO encapuslates ALL of the subsequent buffers and states of the buffers 
	glGenVertexArrays( 1, &vao ); 
	glBindVertexArray( vao );

	glGenBuffers( 1, &dataVBO );
	GL_PEEK_ERRORS_AT

	glBindBuffer( GL_ARRAY_BUFFER, dataVBO );
	GL_PEEK_ERRORS_AT

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ARRAY_BUFFER, sizeof( GLfloat ) * sphereVertices.size(), &sphereVertices[ 0 ], GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT

	glGenBuffers( 1, &indexVBO );
	GL_PEEK_ERRORS_AT
	
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexVBO );
	GL_PEEK_ERRORS_AT
	
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * sphereIndices.size(), &sphereIndices[ 0 ], GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT
	
	glBindVertexArray( 0 );

	this->m_unitSphere = new Mesh( vao, dataVBO, indexVBO, sphereIndices.size() );
	this->m_unitSphere->m_vertexData = NULL;	// no dynamic memory for small mesh 
	this->m_unitSphere->m_indexData = NULL;		// no dynamic memory for small mesh 

	// put into map => will be cleaned up
	MeshNode* containerNode = new MeshNode( "UNIT_SPHERE" );
	containerNode->m_meshes.push_back( this->m_unitSphere );

	this->m_allMeshes[ containerNode->getName() ] = containerNode;

	return this->m_unitSphere;
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

	// TODO: add aiProcess_LimitBoneWeights will default max bones to 4 / vertex
	// TODO: add aiProcess_ImproveCacheLocality to improve cache-access patterns for vertex-transformation but has O(n) complexity

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
	rootNode = GeometryFactory::processNode( this->m_currentScene->mRootNode, glm::mat4() );

	aiReleaseImport( this->m_currentScene );
	this->m_currentScene = NULL;
	this->m_currentBonesHierarchical.clear();

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;

    return rootNode;
}

MeshNode*
GeometryFactory::processNode( const struct aiNode* assImpNode, const glm::mat4& parentGlobalTransform )
{
	MeshNode* node = new MeshNode( assImpNode->mName.C_Str() );
	AssImpUtils::assimpMatToGlm( assImpNode->mTransformation, node->m_localTransform );
	node->m_globalTransform = parentGlobalTransform * node->m_localTransform;

	// find matching bone to this hierarchy-node, is optional!
	int boneIndex = this->getBoneIndex( node->m_name );
	if ( -1 != boneIndex )
	{
		node->m_boneOffset = new glm::mat4( this->m_currentBonesHierarchical[ boneIndex ].m_offset );
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

	node->m_hasMeshesFlag = node->m_meshes.size() != 0;

	// walk tree down recursive
	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		MeshNode* childNode = this->processNode( assImpNode->mChildren[ i ], node->m_globalTransform );
		node->compareAndSetBB( childNode->getBBMin(), childNode->getBBMax() );
		node->m_children.push_back( childNode );

		if ( childNode->m_hasMeshesFlag )
		{
			node->m_hasMeshesFlag = true;
		}
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

Mesh*
GeometryFactory::processMeshBoned( const struct aiMesh* assImpMesh )
{
	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	GLuint* indexData = NULL;
	BonedVertexData* vertexData = NULL;

	// allocate vertexdata
	vertexData = new BonedVertexData[ assImpMesh->mNumVertices ];
	memset( vertexData, 0, assImpMesh->mNumVertices * sizeof( BonedVertexData ) );

	// indexbuffer for faces
	indexData = new GLuint[ assImpMesh->mNumFaces * 3 ];
	memset( indexData, 0, assImpMesh->mNumFaces * 3 * sizeof( GLuint ) );

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
			BonedVertexData& vertex = vertexData[ newWeight.mVertexId ];

			GeometryFactory::processBoneWeight( vertex, newWeight, boneIndex );
		}
	}

	for ( unsigned int i = 0; i < assImpMesh->mNumFaces; ++i )
	{
		const struct aiFace* face = &assImpMesh->mFaces[ i ];

		for( unsigned int j = 0; j < face->mNumIndices; j++ )
		{
			int index = face->mIndices[ j ];
			float boneWeightSum = 0.0f;

			// normalize normals during load-time to ensure they arrive normalized in shader
			assImpMesh->mNormals[ index ].Normalize();

			indexData[ i * 3 + j ] = index;
			memcpy( vertexData[ index ].position, &assImpMesh->mVertices[ index ].x, sizeof( Vertex ) );
			memcpy( vertexData[ index ].normal, &assImpMesh->mNormals[ index ].x, sizeof( Normal ) );

			if ( assImpMesh->HasTangentsAndBitangents() )
			{
				// normalize tangents during load-time to ensure they arrive normalized in shader
				assImpMesh->mTangents[ index ].Normalize();
				memcpy( vertexData[ index ].tangent, &assImpMesh->mTangents[ index ].x, sizeof( Tangent ) );
			}

			if ( assImpMesh->HasTextureCoords( 0 ) )
			{
				vertexData[ index ].texCoord[ 0 ] = assImpMesh->mTextureCoords[ 0 ][ index ].x;
				vertexData[ index ].texCoord[ 1 ] = assImpMesh->mTextureCoords[ 0 ][ index ].y;
			}

			unsigned int minWeightIndex = 0;

			// adjust bone-weights to 1.0 because if more than 4 bones influenced this vertex we only choose the largest 4 weights
			for ( unsigned int k = 0; k < vertexData[ index ].boneCount; k++ )
			{
				boneWeightSum += vertexData[ index ].boneWeights[ k ];

				if ( vertexData[ index ].boneWeights[ k ] < vertexData[ index ].boneWeights[ minWeightIndex ] )
				{
					minWeightIndex = k;
				}
			}

			// we can safely add the difference to the smallest weight
			float weightsDiff = 1.0f - boneWeightSum;
			vertexData[ index ].boneWeights[ minWeightIndex ] += weightsDiff;

			GeometryFactory::updateBB( assImpMesh->mVertices[ index ], meshBBmin, meshBBmax );
		}
	}

	GLuint vao = 0;
	GLuint dataVBO = 0;
	GLuint indexVBO = 0;

	// NOTE: the VAO encapuslates ALL of the subsequent buffers and states of the buffers 
	glGenVertexArrays( 1, &vao ); 
	glBindVertexArray( vao );

	// generate and setup data vbo
	glGenBuffers( 1, &dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ARRAY_BUFFER, dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ARRAY_BUFFER, sizeof( BonedVertexData ) * assImpMesh->mNumVertices, vertexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to coordinates
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to normals
	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to texture coords
	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 24 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 3 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to tangent
	glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 32 ) );
	GL_PEEK_ERRORS_AT_DEBUG
	
	glEnableVertexAttribArray( 4 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to bone-count
	glVertexAttribIPointer( 4, 1, GL_UNSIGNED_INT, sizeof( BonedVertexData ), BUFFER_OFFSET( 44 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 5 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to bone-indices 
	glVertexAttribIPointer( 5, 4, GL_UNSIGNED_INT, sizeof( BonedVertexData ), BUFFER_OFFSET( 48 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 6 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to bone-weights
	glVertexAttribPointer( 6, 4, GL_FLOAT, GL_FALSE, sizeof( BonedVertexData ), BUFFER_OFFSET( 64 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// generate and setup index vbo
	glGenBuffers( 1, &indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * assImpMesh->mNumFaces * 3, indexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindVertexArray( 0 );

	Mesh* meshBoned = new Mesh( vao, dataVBO, indexVBO, assImpMesh->mNumFaces * 3 );
	// TODO: no need to store the vertex- & index-data as it is hold by OpenGL, can be deleted
	meshBoned->m_vertexData = vertexData;
	meshBoned->m_indexData = indexData;

	meshBoned->setBB( meshBBmin, meshBBmax );

	return meshBoned;
}

void
GeometryFactory::processBoneWeight( BonedVertexData& vertex, const aiVertexWeight& newWeight, unsigned int boneIndex )
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

		for ( unsigned int i = 1; i < vertex.boneCount; i++ )
		{
			if ( vertex.boneWeights[ i ] < vertex.boneWeights[ minWeightIndex ] )
			{
				minWeightIndex = i;
			}
		}

		// the new bone has more influence on this vertex then an existing one, replace the existing one
		if ( newWeight.mWeight > vertex.boneWeights[ minWeightIndex ] )
		{
			vertex.boneIndices[ minWeightIndex ] = boneIndex;
			vertex.boneWeights[ minWeightIndex ] = newWeight.mWeight;
		}
	}
}

Mesh*
GeometryFactory::processMeshStatic( const struct aiMesh* assImpMesh )
{
	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	GLuint* indexData = NULL;
	StaticVertexData* vertexData = NULL;

	meshBBmin[ 0 ] = numeric_limits<float>::max();
	meshBBmin[ 1 ] = numeric_limits<float>::max();
	meshBBmin[ 2 ] = numeric_limits<float>::max();

	meshBBmax[ 0 ] = numeric_limits<float>::min();
	meshBBmax[ 1 ] = numeric_limits<float>::min();
	meshBBmax[ 2 ] = numeric_limits<float>::min();

	// allocate vertexdata
	vertexData = new StaticVertexData[ assImpMesh->mNumVertices ];
	memset( vertexData, 0, assImpMesh->mNumVertices * sizeof( StaticVertexData ) );

	// indexbuffer for faces
	indexData = new GLuint[ assImpMesh->mNumFaces * 3 ];
	memset( indexData, 0, assImpMesh->mNumFaces * 3 * sizeof( GLuint ) );

	for ( unsigned int i = 0; i < assImpMesh->mNumFaces; ++i )
	{
		const struct aiFace* face = &assImpMesh->mFaces[ i ];

		for( unsigned int j = 0; j < face->mNumIndices; j++ )
		{
			int index = face->mIndices[ j ];

			// normalize normals during load-time to ensure they arrive normalized in shader
			assImpMesh->mNormals[ index ].Normalize();

			indexData[ i * 3 + j ] = index;
			memcpy( vertexData[ index ].position, &assImpMesh->mVertices[ index ].x, sizeof( Vertex ) );
			memcpy( vertexData[ index ].normal, &assImpMesh->mNormals[ index ].x, sizeof( Normal ) );
			
			if ( assImpMesh->HasTangentsAndBitangents() )
			{
				// normalize tangents during load-time to ensure they arrive normalized in shader
				assImpMesh->mTangents[ index ].Normalize();
				memcpy( vertexData[ index ].tangent, &assImpMesh->mTangents[ index ].x, sizeof( Tangent ) );
			}

			if ( assImpMesh->HasTextureCoords( 0 ) )
			{
				vertexData[ index ].texCoord[ 0 ] = assImpMesh->mTextureCoords[ 0 ][ index ].x;
				vertexData[ index ].texCoord[ 1 ] = assImpMesh->mTextureCoords[ 0 ][ index ].y;
			}

			GeometryFactory::updateBB( assImpMesh->mVertices[ index ], meshBBmin, meshBBmax );
		}
	}

	GLuint vao = 0;
	GLuint dataVBO = 0;
	GLuint indexVBO = 0;

	// NOTE: the VAO encapuslates ALL of the subsequent buffers and states of the buffers 
	glGenVertexArrays( 1, &vao ); 
	glBindVertexArray( vao );

	// generate and setup data vbo
	glGenBuffers( 1, &dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ARRAY_BUFFER, dataVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ARRAY_BUFFER, sizeof( StaticVertexData ) * assImpMesh->mNumVertices, vertexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to coordinates
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 0 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 1 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to normals
	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 12 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 2 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to texture coords
	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 24 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	glEnableVertexAttribArray( 3 );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify pointer to tangent
	glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, sizeof( StaticVertexData ), BUFFER_OFFSET( 32 ) );
	GL_PEEK_ERRORS_AT_DEBUG

	// generate and setup index vbo
	glGenBuffers( 1, &indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, indexVBO );
	GL_PEEK_ERRORS_AT_DEBUG

	glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( GLuint ) * assImpMesh->mNumFaces * 3, indexData, GL_STATIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

	glBindVertexArray( 0 );

	Mesh* meshStatic = new Mesh( vao, dataVBO, indexVBO, assImpMesh->mNumFaces * 3 );
	// TODO: no need to store the vertex- & index-data as it is hold by OpenGL, can be deleted
	meshStatic->m_vertexData = vertexData;
	meshStatic->m_indexData = indexData;

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
				MeshBone bone;
				bone.m_name = assImpBone->mName.C_Str();
				AssImpUtils::assimpMatToGlm( assImpBone->mOffsetMatrix, bone.m_offset );
				
				// bones are stored in row-major order => transpose it
				bone.m_offset = glm::transpose( bone.m_offset );

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
