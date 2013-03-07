/*
 *  GeometryFactoryLegacy.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 01.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GeometryFactoryLegacyLegacy.h"

#include "GeomMesh.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/types.h> 
 #include <sys/stat.h> 
 #include <unistd.h> 

#include "Loaders/Ply/ply.h"

#include <glm/gtc/type_ptr.hpp>


extern "C" {
#include <lib3ds.h>
}

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>

// byte-align structures
#ifdef _MSC_VER
#   pragma pack( push, packing )
#   pragma pack( 1 )
#   define PACK_STRUCT
#elif defined( __GNUC__ )
#   define PACK_STRUCT __attribute__((packed))
#else /* not __GNUC__ */
#   error you must byte-align these structures with the appropriate compiler dir
#endif /* not __GNUC__ */

#ifndef byte
	typedef unsigned char byte;
#endif /* not byte */

#ifndef word
	typedef unsigned short word;
#endif /* not word */

// Header structure
struct MilkshapeHeaderStruct 
{
	char id[10];
	word version;
} PACK_STRUCT;
typedef struct MilkshapeHeaderStruct MilkshapeHeader;

// Vertex structure
struct MilkshapeVertexStruct
{
	byte flags;
	float vertex[3];
	char boneId;
	byte referenceCount;
} PACK_STRUCT;
typedef struct MilkshapeVertexStruct MilkshapeVertex;

// Triangle3D structure
struct MilkshapeTriangleStruct
{
	word flags;
	word vertexIndices[3];
	float vertexNormals[3][3];
	float s[3];
	float t[3];
	byte smoothingGroup;
	byte groupIndex;
} PACK_STRUCT;

typedef struct MilkshapeTriangleStruct MilkshapeTriangle;

// Group structure
struct MilkshapeGroupStruct
{
	byte flags;
	char name[32];
	word numtriangles;
	word* triangleIndices;
	signed char materialIndex; // -1 = no material
} PACK_STRUCT;

typedef struct MilkshapeGroupStruct MilkshapeGroup;

// Material structure
struct MilkshapeMaterialStruct
{
	char name[32];
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float emissive[4];
	float shininess; // 0.0f - 128.0f
	float transparency; // 0.0f - 1.0f
	char mode; // 0, 1, 2 is unused now
	char texture[128]; // texture.bmp
	char alphamap[128]; // alpha.bmp
} PACK_STRUCT;

typedef struct MilkshapeMaterialStruct MilkshapeMaterial;

// Default alignment
#ifdef _MSC_VER
#   pragma pack( pop, packing )
#endif /* _MSC_VER */

#undef PACK_STRUCT

struct PlyVertex
{
	float x,y,z;
	float nx,ny,nz;
};

struct PlyFace
{
	unsigned char nverts;
	int* vertexIndices;
};

PlyProperty ply_vert_props[] =
{
  {"x", Float32, Float32, offsetof(PlyVertex, x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(PlyVertex, y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(PlyVertex, z), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(PlyVertex, nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(PlyVertex, ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(PlyVertex,nz), 0, 0, 0, 0}
};


PlyProperty ply_face_props[] =
{
  {"vertex_indices", Int32, Int32, offsetof(PlyFace, vertexIndices), PLY_LIST, Uint8, Uint8, offsetof(PlyFace, nverts)},
};

float
abs(float a)
{
	return a < 0 ? -a : a;
}

using namespace std;

map<string, GeomType*> GeometryFactoryLegacy::meshes;

GeomType*
GeometryFactoryLegacy::get( const std::string& fileName )
{
	map<std::string, GeomType*>::iterator findIter = GeometryFactoryLegacy::meshes.find( fileName );
	if ( findIter != GeometryFactoryLegacy::meshes.end() )
		return findIter->second;

	string ending;
	unsigned long index = fileName.find_last_of('.');
	if (index != string::npos)
		ending = fileName.substr(index + 1, fileName.length() - index);

	GeomType* geomType = 0;

	if (strcasecmp(ending.c_str(), "ply") == 0)
	{
		geomType = GeometryFactoryLegacy::loadPly(fileName);
	}
	else if (strcasecmp(ending.c_str(), "ms3d") == 0)
	{
		geomType = GeometryFactoryLegacy::loadMs3D(fileName);
	}
	else if (strcasecmp(ending.c_str(), "3ds") == 0)
	{
		geomType = GeometryFactoryLegacy::load3DS(fileName);
	}
	else
	{
		geomType = GeometryFactoryLegacy::loadFolder(fileName);
	}

	if ( geomType )
		GeometryFactoryLegacy::meshes[ fileName ] = geomType;

	return geomType;
}

void
GeometryFactoryLegacy::freeAll()
{
	map<std::string, GeomType*>::iterator iter = GeometryFactoryLegacy::meshes.begin();
	while ( iter != GeometryFactoryLegacy::meshes.end() )
	{
		GeomType* geom = iter->second;
		delete geom;

		iter++;
	}

	GeometryFactoryLegacy::meshes.clear();
}

GeomType*
GeometryFactoryLegacy::loadFolder(const std::string& folderName)
{
	string fullPath = folderName;
	DIR* directory = opendir(fullPath.c_str());
	if (!directory)
	{
			cout << "ERROR ... couldn't open Directory \"" << fullPath << "\" in GeometryFactoryLegacy::loadFolder" << endl;
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

		GeomType* subFolderGeometryFactoryLegacy = 0;
		string subFolderPath = folderName + "/" + fileName;

		if (S_ISDIR(entryStatus.st_mode))
		{
			//cout << "subFolderPath = " << subFolderPath << endl;
			subFolderGeometryFactoryLegacy = GeometryFactoryLegacy::loadFolder(subFolderPath);

		}
		else
		{
			string ending;
			unsigned long index = fullFileName.find_last_of('.');
			if (index != string::npos)
				ending = fullFileName.substr(index + 1, fullFileName.length() - index);

			//cout << "subFilePath = " << fullFileName << endl;

			if (strcasecmp(ending.c_str(), "ply") == 0) {
				subFolderGeometryFactoryLegacy = GeometryFactoryLegacy::loadPly(fullFileName);
			} else if (strcasecmp(ending.c_str(), "ms3d") == 0) {
				subFolderGeometryFactoryLegacy = GeometryFactoryLegacy::loadMs3D(fullFileName);
			} else if (strcasecmp(ending.c_str(), "3ds") == 0) {
				subFolderGeometryFactoryLegacy = GeometryFactoryLegacy::load3DS(fullFileName);
			} else {
				cout << "bad ending: \"" << ending << "\"" << endl;
			}
		}
		
		if (subFolderGeometryFactoryLegacy) {
			subFolderGeometryFactoryLegacy->parent = folderGroup;
			
			folderGroup->compareBB(subFolderGeometryFactoryLegacy->getBBMin(), subFolderGeometryFactoryLegacy->getBBMin());
			folderGroup->children.push_back(subFolderGeometryFactoryLegacy);
		}
	}

	closedir(directory);

	return folderGroup;
}

GeomType*
GeometryFactoryLegacy::load3DS(const std::string& fileName)
{
	glm::vec3 geomGroupBBmin;
	glm::vec3 geomGroupBBmax;

	Lib3dsFile* file = 0;
	GeomType* geomGroup = 0;

	string fullFilename = fileName;

	file = lib3ds_file_open( fullFilename.c_str() );
	if ( 0 == file )
	{
		cout << "ERROR ... couldnt load GeometryFactoryLegacy " << fullFilename << endl;
		return 0;
	}

	cout << "LOADING ... " << fileName << endl;

	if ( file->nmeshes > 1 )
	{
		geomGroup = new GeomType();
		geomGroup->name = file->name;
	}

	lib3ds_file_create_nodes_for_meshes( file );

	for ( Lib3dsNode* node = file->nodes; node != 0; node = node->next )
	{
		if ( node->type == LIB3DS_NODE_MESH_INSTANCE )
		{
			Lib3dsMesh* mesh = lib3ds_file_mesh_for_node( file, node );
			if ( NULL == mesh )
			{
				cout << "Corrupted file \"" << fileName << "\" : coulnd't find mesh " << node->name << endl;
				continue;
			}

			// indexbuffer for faces
			GLuint* indexBuffer = new GLuint[ mesh->nfaces * 3 ];
			memset( indexBuffer, 0, mesh->nfaces * 3 * sizeof( GLuint ) );

			// we cannot use vertexData for normals because they have to calculated first
			GeomMesh::Normal* normals = new GeomMesh::Normal[ mesh->nfaces * 3 ];
			memset( normals, 0, mesh->nfaces * 3 * sizeof( GeomMesh::Normal ) );

			// allocate vertexdata
			GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ mesh->nvertices ];
			memset( vertexData, 0, mesh->nvertices * sizeof( GeomMesh::VertexData ) );

			glm::vec3 meshBBmin;
			glm::vec3 meshBBmax;
			lib3ds_mesh_bounding_box( mesh, glm::value_ptr( meshBBmin ), glm::value_ptr( meshBBmax ) );

			if ( file->nmeshes > 1 )
			{
				for ( int i = 0; i < 3; i++ )
				{
					if (meshBBmin[ i ] < geomGroupBBmin[ i ])
						geomGroupBBmin[ i ] = meshBBmin[ i ];
					else if (meshBBmax[ i ] > geomGroupBBmax[ i ])
						geomGroupBBmax[ i ] = meshBBmax[ i ];
				}
			}

			for ( unsigned int i = 0; i < mesh->nvertices; i++ )
			{
				/*
				for ( unsigned int j = 0; j < 3; j++ )
				{
					vertexData[ i ].position[ j ] = mesh->vertices[ i ][ j ];
					//vertexData[ i ].normal[ j ] = normals[ i ][ j ];
				}
				 */

				// flip z and y achsis
				vertexData[ i ].position[ 0 ] = mesh->vertices[ i ][ 0 ];
				vertexData[ i ].position[ 1 ] = mesh->vertices[ i ][ 2 ];
				vertexData[ i ].position[ 2 ] = mesh->vertices[ i ][ 1 ];

				vertexData[ i ].normal[ 0 ] = normals[ i ][ 0 ];
				vertexData[ i ].normal[ 1 ] = normals[ i ][ 2 ];
				vertexData[ i ].normal[ 2 ] = normals[ i ][ 1 ];
			}
	
			lib3ds_mesh_calculate_vertex_normals( mesh, normals );

			for( unsigned int i = 0; i < mesh->nfaces; i++ )
			{
				Lib3dsFace* face = &mesh->faces[ i ];

				for(unsigned int j = 0; j < 3; j++)
				{
					indexBuffer[ i * 3 + j ] = face->index[ j ];
					vertexData[ face->index[ j ] ].normal[ j ] = normals[ i * 3 + j ][ j ];
				}
			}

			delete[] normals;

			GeomMesh* geomMesh = new GeomMesh( mesh->nfaces, mesh->nvertices, vertexData, indexBuffer );
			geomMesh->setBB(meshBBmin, meshBBmax);
			geomMesh->name = node->name;

			Lib3dsMeshInstanceNode* typedNode = (Lib3dsMeshInstanceNode*) node;

			glm::mat4 transl;
			glm::mat4 meshTransf;
			glm::mat4 instanceTransf;
			float tmpMat[4][4];

			 // need to invert to make openGL compatible
			lib3ds_matrix_copy( tmpMat, mesh->matrix );
			lib3ds_matrix_inv( tmpMat );
			memcpy( glm::value_ptr( meshTransf ), &tmpMat[ 0 ][ 0 ], sizeof(float) * 16 );

			lib3ds_matrix_copy( tmpMat, typedNode->base.matrix );
			lib3ds_matrix_inv( tmpMat );
			memcpy( glm::value_ptr( instanceTransf ), &tmpMat[ 0 ][ 0 ], sizeof(float) * 16 );

			glm::value_ptr( transl )[ 12 ] = typedNode->pivot[ 0 ];
			glm::value_ptr( transl )[ 13 ] = typedNode->pivot[ 1 ];
			glm::value_ptr( transl )[ 14 ] = typedNode->pivot[ 2 ];

			geomMesh->m_modelMatrix = transl;

			if ( file->nmeshes > 1 )
			{
				geomGroup->children.push_back( geomMesh );
			}
			else
			{
				geomGroup = geomMesh;
			}
		}
		else
		{
			cout << "unsupported node type" << endl;
		}
	}
	
	if ( file->nmeshes > 1 )
		geomGroup->setBB(geomGroupBBmin, geomGroupBBmax);
	
    lib3ds_file_free( file );

    cout << "LOADED ... " << fileName << endl;
    
    return geomGroup;
}

GeomType*
GeometryFactoryLegacy::loadMs3D(const std::string& fileName)
{
	GeomType* geom = 0;

	GLuint* indexBuffer = 0;
	GeomMesh::VertexData* vertexData = 0;

	word numVertices = 0;
	word numTriangles = 0;

	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

	string fullFilename = fileName;
	ifstream fileStream(fullFilename.c_str(), ios::in | ios::binary);
	
	cout << "LOADING ... " << fileName << endl;

	fileStream.seekg( 0, ios::end );
	long fileSize = fileStream.tellg();
	fileStream.seekg( 0, ios::beg );

	char* buffer = (char*) malloc( sizeof(char) * fileSize );
	fileStream.read(buffer, fileSize);
	fileStream.close();

	const byte* bufferPointer = (byte*) buffer;
	MilkshapeHeader* header = (MilkshapeHeader*) bufferPointer;

	if (strncmp("MS3D000000", header->id, 10) != 0) {
		cout << "Not a milkshape file" << endl;
		return 0;
	}

	if (header->version < 3 || header->version > 4) {
		cout << "Unsupported file version" << endl;
		return 0;
	}
			
	bufferPointer += 14;

	numVertices = *((word*) bufferPointer);
	bufferPointer += 2;

	vertexData = new GeomMesh::VertexData[ numVertices ];
	memset( vertexData, 0, sizeof( GeomMesh::VertexData ) * numVertices );
	
	for (int i = 0; i < numVertices; i++)
	{
		MilkshapeVertex* vertex = (MilkshapeVertex*) bufferPointer;

		for ( int j = 0; j < 3; j++ )
		{
			if (vertex->vertex[ j ] > meshBBmax[ j ])
				meshBBmax[ j ] = vertex->vertex[ j ];
			else if (vertex->vertex[ j ] < meshBBmin[ j ])
				meshBBmin[ j ] = vertex->vertex[ j ];

			vertexData[ i ].position[ j ] = vertex->vertex[ j ];
		}
		
		bufferPointer += sizeof( MilkshapeVertex );
	}

	numTriangles = *((word*) bufferPointer);
	bufferPointer += 2;

	indexBuffer = new GLuint[ numTriangles * 3 ];
	memset( indexBuffer, 0, sizeof( GLuint ) * numTriangles * 3 );

	for (int i = 0; i < numTriangles; i++)
	{
		MilkshapeTriangle* triangle = (MilkshapeTriangle*) bufferPointer;

		for ( int j = 0; j < 3; j++ )
		{
			indexBuffer[ i * 3 + j ] = triangle->vertexIndices[ j ];
			//vertexData[ i * 3 + j ].normal[ j ] = triangle->vertexNormals[ j ][ j ];
		}

		bufferPointer += sizeof( MilkshapeTriangle );
	}
	
	geom = new GeomMesh( numVertices, numTriangles, vertexData, indexBuffer );
	geom->setBB( meshBBmin, meshBBmax );


/*
	word numGroups = *((word*) bufferPointer);
	bufferPointer += 2;
	
	MilkshapeGroup** groups = new MilkshapeGroup*[numGroups];

	for (int i = 0; i < numGroups; i++)
	{
		groups[i] = new MilkshapeGroup;
		memcpy(groups[i], bufferPointer, 35);
		bufferPointer += 35;

		groups[i]->triangleIndices = (word*) malloc(2 * groups[i]->numtriangles);
		memcpy(groups[i]->triangleIndices, bufferPointer, 2 * groups[i]->numtriangles);
		bufferPointer += 2 * groups[i]->numtriangles;

		memcpy(&groups[i]->materialIndex, bufferPointer, 1);
		bufferPointer += 1;
	}

	word numMaterials = *((word*) bufferPointer);
	bufferPointer += 2;

	MilkshapeMaterial* materials = new MilkshapeMaterial[ numMaterials ];
	
	for ( int i = 0; i < numMaterials; i++ )
	{
		MilkshapeMaterial* material = ( MilkshapeMaterial* ) bufferPointer;
		memcpy( &materials[ i ], material, sizeof( MilkshapeMaterial ) );
		bufferPointer += sizeof( MilkshapeMaterial );
	}

	GeomType* geomGroup = 0;
	
	if (numGroups > 1)
		geomGroup = new GeomType();

	for (int i = 0; i < numGroups; i++)
	{
		int numTriangles = groups[i]->numtriangles;
		
		GLuint* indexBuffer = new GLuint[ numTriangles * 3 ];
		memset( indexBuffer, 0, sizeof( GLuint ) * numTriangles * 3 );

		GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ numVertices ];
		memset( vertexData, 0, sizeof( GeomMesh::VertexData ) * numVertices );

		Vector meshBBmin;
		Vector meshBBmax;

		for (int j = 0; j < groups[i]->numtriangles; j++)
		{
			MilkshapeTriangle& triangle = triangles[groups[i]->triangleIndices[j]];
			
			for (int k = 0; k < 3; k++)
			{
				MilkshapeVertex& vertex = vertices[triangle.vertexIndices[k]];
				         
				if (vertex.vertex[0] > meshBBmax[0])
					meshBBmax.data[0] = vertex.vertex[0];
				else if (vertex.vertex[0] < meshBBmin[0])
					meshBBmin.data[0] = vertex.vertex[0];
				
				if (vertex.vertex[1] > meshBBmax[1])
					meshBBmax.data[1] = vertex.vertex[1];
				else if (vertex.vertex[1] < meshBBmin[1])
					meshBBmin.data[1] = vertex.vertex[1];
				
				if (vertex.vertex[2] > meshBBmax[2])
					meshBBmax.data[2] = vertex.vertex[2];
				else if (vertex.vertex[2] < meshBBmin[2])
					meshBBmin.data[2] = vertex.vertex[2];

				vertexData[ j * 3 + k ].position[0] = vertex.vertex[0];
				vertexData[ j * 3 + k ].position[1] = vertex.vertex[1];
				vertexData[ j * 3 + k ].position[2] = vertex.vertex[2];
				
				vertexData[ j * 3 + k ].normal[0] = triangle.vertexNormals[k][0];
				vertexData[ j * 3 + k ].normal[1] = triangle.vertexNormals[k][1];
				vertexData[ j * 3 + k ].normal[2] = triangle.vertexNormals[k][2];
			}
		}
		
		GeomMesh* geomMesh = new GeomMesh( numTriangles, 0, vertexData, indexBuffer );
		geomMesh->setBB(meshBBmin, meshBBmax);
		
		if (numGroups > 1)
			geomGroup->children.push_back(geomMesh);
		else
			geomGroup = geomMesh;
	}
	
	if (numGroups > 1)
		geomGroup->setBB(geomGroupBBmin, geomGroupBBmax);
	*/

	/*
	delete[] materials;
	delete[] groups;
	delete[] vertices;
	delete[] triangles;
	*/

	free( buffer );
	
	cout << "LOADED ... " << fileName << endl;

	return geom;
}

// TODO: rework, not properly working?
GeomType*
GeometryFactoryLegacy::loadPly(const std::string& fileName)
{	
	string fullFilename = fileName;

	cout << "LOADING ... " << fileName << endl;

	FILE* file = fopen(fileName.c_str(), "rb");
	if (file == 0) {
		cout << "ERROR in GeometryFactoryLegacy: failed opening file \"" << fileName << "\" with error " << endl;
		return 0;
	}

	PlyFile* plyFile = read_ply(file);
	if (plyFile == 0) {
		cout << "ERROR in GeometryFactoryLegacy: failed reading file \"" << fileName << "\"" << endl;
		return 0;
	}

	unsigned int faceCount = 0;

    vector<PlyFace> faces;
    vector<PlyVertex> vertices;

    for (int i = 0; i < plyFile->num_elem_types; i++) {
    	int elemCount = 0;
    	char* elem_name = setup_element_read_ply(plyFile, i, &elemCount);

    	if (strcasecmp ("vertex", elem_name) == 0)
    	{
    		ply_get_property (plyFile, elem_name, &ply_vert_props[0]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[1]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[2]);

		    ply_get_property (plyFile, elem_name, &ply_vert_props[3]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[4]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[5]);

		    for (int j = 0; j < elemCount; j++)
		    {
				PlyVertex vertex;
		        ply_get_element (plyFile, (void *) &vertex);
		        vertices.push_back(vertex);

		        //cout << "vertex " << j << " at (" << vertex.x << "/" << vertex.y << "/" << vertex.z << ")" << endl;
		    }

    	}
    	else if (strcasecmp ("face", elem_name) == 0)
    	{
    		faceCount = elemCount;
    	    ply_get_property (plyFile, elem_name, &ply_face_props[0]);

    	    for (unsigned int j = 0; j < faceCount; j++)
    	    {
    	    	PlyFace plyFace;
    	    	plyFace.vertexIndices = 0;
    	    	ply_get_element(plyFile, (void*) &plyFace);

    	    	/*
    	    	for (int k = 0; k < plyFace.nverts; k++) {
    	    		plyFace.vertexIndices[k] = htonl(plyFace.vertexIndices[k]);
    	    		//cout << "face has index of " << k << ". vertex at " << plyFace.vertexIndices[k] << endl;
    	    	}
*/
    	    	faces.push_back(plyFace);
    	    }
    	}
    }

    close_ply(plyFile);
	free_ply(plyFile);

	glm::vec3 meshBBmin;
	glm::vec3 meshBBmax;

    cout << "we got " << faceCount << " faces and " << vertices.size() << " vertices " << endl;

    GLuint* indexBuffer = new GLuint[ faceCount * 3 ];
	GeomMesh::VertexData* vertexData = new GeomMesh::VertexData[ faceCount * 3 ];

  	for (unsigned int i = 0; i < faceCount; i++)
  	{
		PlyFace face = faces[i];

		//cout << "face " << i << " has " << face.nverts << " vertices" << endl;

		for (int j = 0; j < 3; j++)
		{
			int index = htonl( face.vertexIndices[j] );
			//cout << "index = " << index << endl;
			indexBuffer[ i * 3 + j ] = index;

			PlyVertex vertex = vertices[ index ];

			float x = vertex.x;
			float y = vertex.y;
			float z = vertex.z;

			float nx = vertex.nx;
			float ny = vertex.ny;
			float nz = vertex.nz;

			if (x > meshBBmax[0])
				meshBBmax[0] = x;
			else if (x < meshBBmin[0])
				meshBBmin[0] = x;

			if (y > meshBBmax[1])
				meshBBmax[1] = y;
			else if (y < meshBBmin[1])
				meshBBmin[1] = y;

			if (z > meshBBmax[2])
				meshBBmax[2] = z;
			else if (z < meshBBmin[2])
				meshBBmin[2] = z;

			vertexData[ i * 3 + j ].position[ 0 ] = x;
			vertexData[ i * 3 + j ].position[ 1 ] = y;
			vertexData[ i * 3 + j ].position[ 2 ] = z;

			vertexData[ i * 3 + j ].normal[ 0] = nx;
			vertexData[ i * 3 + j ].normal[ 1 ] = ny;
			vertexData[ i * 3 + j ].normal[ 2 ] = nz;
		}
	}

  	GeomMesh* geomMesh = new GeomMesh( faceCount, 0, vertexData, indexBuffer );
  	geomMesh->setBB(meshBBmin, meshBBmax);
	
	return geomMesh;
}
