/*
 *  model.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 01.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "Model.h"

#include "geom/GeomMesh.h"
#include "geom/GeomTeapot.h"
#include "geom/GeomPlane.h"

#include <arpa/inet.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/types.h> 
 #include <sys/stat.h> 
 #include <unistd.h> 

#include "geom/loaders/ply/ply.h"

#include <lib3ds/file.h>
#include <lib3ds/mesh.h>

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

struct PlyVertex {
	float x,y,z;
	float nx,ny,nz;
};

struct PlyFace {
	unsigned char nverts;
	int* vertexIndices;
};

PlyProperty ply_vert_props[] = {
  {"x", Float32, Float32, offsetof(PlyVertex, x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(PlyVertex, y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(PlyVertex, z), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(PlyVertex, nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(PlyVertex, ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(PlyVertex,nz), 0, 0, 0, 0}
};


PlyProperty ply_face_props[] = {
  {"vertex_indices", Int32, Int32, offsetof(PlyFace, vertexIndices), PLY_LIST, Uint8, Uint8, offsetof(PlyFace, nverts)},
};

float
abs(float a)
{
	return a < 0 ? -a : a;
}

using namespace std;

map<string, GeomType*> Model::models;

GeomType* Model::get(const std::string& id)
{
	map<std::string, GeomType*>::iterator findIter = Model::models.find(id);
	if (findIter != Model::models.end())
		return findIter->second;

	return 0;
}

void Model::loadMesh(const std::string& id, const std::string& fileName)
{
	string ending;
	unsigned long index = fileName.find_last_of('.');
	if (index != string::npos)
		ending = fileName.substr(index + 1, fileName.length() - index);

	GeomType* model = 0;

	if (strcasecmp(ending.c_str(), "ply") == 0) {
		model = Model::loadPly(fileName);
	} else if (strcasecmp(ending.c_str(), "ms3d") == 0) {
		model = Model::loadMs3D(fileName);
	} else if (strcasecmp(ending.c_str(), "3ds") == 0) {
		model = Model::load3DS(fileName);
	} else {
		model = Model::loadFolder(fileName);
	}

	if (model)
		Model::models[id] = model;
}
void Model::freeAll()
{
}

GeomType* Model::loadFolder(const std::string& folderName)
{
	string fullPath = "resources/models/" + folderName;	
	DIR* directory = opendir(fullPath.c_str());
	if (!directory) {
			cout << "ERROR ... couldn't open Directory \"" << fullPath << "\" in Model::loadFolder" << endl;		
			return 0;	
	}
	
	GeomType* folderGroup = new GeomType();
	
	struct dirent* entry;
	struct stat entryStatus;

	// iterate through directory
	while ((entry = readdir(directory)) != NULL) {
		string fileName = entry->d_name;
		string fullFileName = fullPath + "/" + fileName;
		
		stat(fullFileName.c_str(), &entryStatus);

		// ignore .
		if(fileName == ".")
			continue;
		
		// ignore
		if (fileName == "..")
			continue;

		GeomType* subFolderModel = 0;
		string subFolderPath = folderName + "/" + fileName;

		if (S_ISDIR(entryStatus.st_mode)) {
			//cout << "subFolderPath = " << subFolderPath << endl;
			subFolderModel = Model::loadFolder(subFolderPath);

		} else {
			string ending;
			unsigned long index = fullFileName.find_last_of('.');
			if (index != string::npos)
				ending = fullFileName.substr(index + 1, fullFileName.length() - index);

			//cout << "subFilePath = " << fullFileName << endl;

			if (strcasecmp(ending.c_str(), "ply") == 0) {
				subFolderModel = Model::loadPly(fullFileName);
			} else if (strcasecmp(ending.c_str(), "ms3d") == 0) {
				subFolderModel = Model::loadMs3D(fullFileName);
			} else if (strcasecmp(ending.c_str(), "3ds") == 0) {
				subFolderModel = Model::load3DS(fullFileName);
			} else {
				cout << "bad ending: \"" << ending << "\"" << endl;
			}
		}
		
		if (subFolderModel) {
			subFolderModel->parent = folderGroup;
			
			folderGroup->compareBB(subFolderModel->getBBMin(), subFolderModel->getBBMin());
			folderGroup->children.push_back(subFolderModel);
		}
	}

	closedir(directory);

	return folderGroup;
}

GeomType* Model::load3DS(const std::string& fileName)
{
	string fullFilename = "resources/models/" + fileName;
	Lib3dsFile* modelData = lib3ds_file_load(fullFilename.c_str());
	if (modelData == 0) {
		cout << "ERROR ... couldnt load model " << fullFilename << endl; 
		return 0;
	}

	cout << "LOADING ... " << fileName << endl;

	int meshCount = 0;
	int totalFaces = 0;
	for(Lib3dsMesh* mesh = modelData->meshes; mesh != NULL;mesh = mesh->next) {
		totalFaces += mesh->faces;
		meshCount++;
	}
	
	GeomType* geomGroup = 0;
	
	Vector geomGroupBBmin;
	Vector geomGroupBBmax;

	if (meshCount > 1)
		geomGroup = new GeomType();
	
	for(Lib3dsMesh* mesh = modelData->meshes; mesh != NULL; mesh = mesh->next) {
		Lib3dsVector* vertices = new Lib3dsVector[mesh->faces * 3];
		Lib3dsVector* normals = new Lib3dsVector[mesh->faces * 3];
		
		Vector meshBBmin;
		Vector meshBBmax;
		lib3ds_mesh_bounding_box(mesh, meshBBmin.data, meshBBmax.data);
		
		if (meshCount > 1) {
			if (meshBBmin[0] < geomGroupBBmin[0])
				geomGroupBBmin.data[0] = meshBBmin[0];
			else if (meshBBmax[0] > geomGroupBBmax[0])
				geomGroupBBmax.data[0] = meshBBmax[0];
			
			if (meshBBmin[1] < geomGroupBBmin[1])
				geomGroupBBmin.data[1] = meshBBmin[1];
			else if (meshBBmax[1] > geomGroupBBmax[1])
				geomGroupBBmax.data[1] = meshBBmax[1];
			
			if (meshBBmin[2] < geomGroupBBmin[2])
				geomGroupBBmin.data[2] = meshBBmin[2];
			else if (meshBBmax[2] > geomGroupBBmax[2])
				geomGroupBBmax.data[2] = meshBBmax[2];
		}
		
		lib3ds_mesh_calculate_normals(mesh, normals);
		
		for(unsigned int i = 0; i < mesh->faces; i++) {
			Lib3dsFace* face = &mesh->faceL[i];

			for(unsigned int j = 0; j < 3; j++) {
				vertices[i * 3 + j][0] = mesh->pointL[face->points[j]].pos[0];
				vertices[i * 3 + j][1] = mesh->pointL[face->points[j]].pos[1];
				vertices[i * 3 + j][2] = mesh->pointL[face->points[j]].pos[2];
				
				//newModel->indices[faceIndex * 3 + j] = face->points[j];
				
				/*
				float tmp = meshStruct->vertices[faceIndex * 3 + j][2];
				meshStruct->vertices[faceIndex * 3 + j][2] = meshStruct->vertices[faceIndex * 3 + j][1];
				meshStruct->vertices[faceIndex * 3 + j][1] = tmp;
				*/
			}
		}
		
		GeomMesh* geomMesh = new GeomMesh(mesh->faces, vertices, normals);
		geomMesh->setBB(meshBBmin, meshBBmax);

		if (meshCount > 1) {
			geomGroup->children.push_back(geomMesh);
		} else {
			geomGroup = geomMesh;
		}
	}
	
	if (meshCount > 1)
		geomGroup->setBB(geomGroupBBmin, geomGroupBBmax);
	
    lib3ds_file_free(modelData);
      
    cout << "LOADED ... " << fileName << endl;
    
    return geomGroup;
}

GeomType* Model::loadMs3D(const std::string& fileName)
{
	string fullFilename = "resources/models/" + fileName;
	ifstream fileStream(fullFilename.c_str(), ios::in | ios::binary);
	
	cout << "LOADING ... " << fileName << endl;

	fileStream.seekg( 0, ios::end );
	long fileSize = fileStream.tellg();
	fileStream.seekg( 0, ios::beg );

	char* buffer = (char*) malloc(sizeof(char) * fileSize);
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

	word numVertices = *((word*) bufferPointer);
	bufferPointer += 2;

	Vector geomGroupBBmin;
	Vector geomGroupBBmax;

	MilkshapeVertex* vertices = new MilkshapeVertex[numVertices];
	
	for (int i = 0; i < numVertices; i++) {
		MilkshapeVertex* vertex = (MilkshapeVertex*) bufferPointer;

		if (vertex->vertex[0] > geomGroupBBmax[0])
			geomGroupBBmax.data[0] = vertex->vertex[0];
		else if (vertex->vertex[0] < geomGroupBBmin[0])
			geomGroupBBmin.data[0] = vertex->vertex[0];

		if (vertex->vertex[1] > geomGroupBBmax[1])
			geomGroupBBmax.data[1] = vertex->vertex[1];
		else if (vertex->vertex[1] < geomGroupBBmin[1])
			geomGroupBBmin.data[1] = vertex->vertex[1];
		
		if (vertex->vertex[2] > geomGroupBBmax[2])
			geomGroupBBmax.data[2] = vertex->vertex[2];
		else if (vertex->vertex[2] < geomGroupBBmin[2])
			geomGroupBBmin.data[2] = vertex->vertex[2];

		memcpy(&vertices[i], vertex, sizeof(MilkshapeVertex));
		
		bufferPointer += sizeof(MilkshapeVertex);
	}

	word numTriangles = *((word*) bufferPointer);
	bufferPointer += 2;

	MilkshapeTriangle* triangles = new MilkshapeTriangle[numTriangles];
	
	for (int i = 0; i < numTriangles; i++) {
		MilkshapeTriangle* triangle = (MilkshapeTriangle*) bufferPointer;
		memcpy(&triangles[i], triangle, sizeof(MilkshapeTriangle));
		bufferPointer += sizeof(MilkshapeTriangle);
	}
	
	word numGroups = *((word*) bufferPointer);
	bufferPointer += 2;
	
	MilkshapeGroup** groups = new MilkshapeGroup*[numGroups];

	for (int i = 0; i < numGroups; i++) {
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

	MilkshapeMaterial* materials = new MilkshapeMaterial[numMaterials];
	
	for (int i = 0; i < numMaterials; i++) {
		MilkshapeMaterial* material = (MilkshapeMaterial*) bufferPointer;
		memcpy(&materials[i], material, sizeof(MilkshapeMaterial));
		bufferPointer += sizeof(MilkshapeMaterial);
	}

	GeomType* geomGroup = 0;
	
	if (numGroups > 1)
		geomGroup = new GeomType();
		
	for (int i = 0; i < numGroups; i++) {
		int numTriangles = groups[i]->numtriangles;
		
		GeomMesh::Vertex* modelVertices = new GeomMesh::Vertex[numTriangles * 3];
		GeomMesh::Vertex* modelNormals = new GeomMesh::Vertex[numTriangles * 3];
		
		Vector meshBBmin;
		Vector meshBBmax;

		for (int j = 0; j < groups[i]->numtriangles; j++) {
			MilkshapeTriangle& triangle = triangles[groups[i]->triangleIndices[j]];
			
			for (int k = 0; k < 3; k++) {
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

				modelVertices[j * 3 + k][0] = vertex.vertex[0];
				modelVertices[j * 3 + k][1] = vertex.vertex[1];
				modelVertices[j * 3 + k][2] = vertex.vertex[2];
				
				modelNormals[j * 3 + k][0] = triangle.vertexNormals[k][0];
				modelNormals[j * 3 + k][1] = triangle.vertexNormals[k][1];
				modelNormals[j * 3 + k][2] = triangle.vertexNormals[k][2];
			}
		}
		
		GeomMesh* geomMesh = new GeomMesh(numTriangles, modelVertices, modelNormals);
		geomMesh->setBB(meshBBmin, meshBBmax);
		
		if (numGroups > 1)
			geomGroup->children.push_back(geomMesh);
		else
			geomGroup = geomMesh;
	}
	
	if (numGroups > 1)
		geomGroup->setBB(geomGroupBBmin, geomGroupBBmax);

	delete[] materials;
	delete[] groups;
	delete[] vertices;
	delete[] triangles;
	delete buffer;
	
	 cout << "LOADED ... " << fileName << endl;

	return geomGroup;
}

GeomType* Model::loadPly(const std::string& fileName)
{	
	string fullFilename = "resources/models/" + fileName;

	cout << "LOADING ... " << fileName << endl;

	FILE* file = fopen(fileName.c_str(), "rb");
	if (file == 0) {
		cout << "ERROR in Model: failed opening file \"" << fileName << "\" with error " << endl;
		return 0;
	}

	PlyFile* plyFile = read_ply(file);
	if (plyFile == 0) {
		cout << "ERROR in Model: failed reading file \"" << fileName << "\"" << endl;
		return 0;
	}

	unsigned int faceCount = 0;

    vector<PlyFace> faces;
    vector<PlyVertex> vertices;

    for (int i = 0; i < plyFile->num_elem_types; i++) {
    	int elemCount = 0;
    	char* elem_name = setup_element_read_ply(plyFile, i, &elemCount);

    	if (strcasecmp ("vertex", elem_name) == 0) {
    		ply_get_property (plyFile, elem_name, &ply_vert_props[0]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[1]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[2]);

		    ply_get_property (plyFile, elem_name, &ply_vert_props[3]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[4]);
		    ply_get_property (plyFile, elem_name, &ply_vert_props[5]);

		    for (int j = 0; j < elemCount; j++) {
				PlyVertex vertex;
		        ply_get_element (plyFile, (void *) &vertex);
		        vertices.push_back(vertex);

		        //cout << "vertex " << j << " at (" << vertex.x << "/" << vertex.y << "/" << vertex.z << ")" << endl;
		    }

    	} else if (strcasecmp ("face", elem_name) == 0) {
    		faceCount = elemCount;
    	    ply_get_property (plyFile, elem_name, &ply_face_props[0]);

    	    for (unsigned int j = 0; j < faceCount; j++) {
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

    Vector meshBBmin;
    Vector meshBBmax;

    cout << "we got " << faceCount << " faces and " << vertices.size() << " vertices " << endl;

	GeomMesh::Vertex* modelVertices = new GeomMesh::Vertex[faceCount * 3];
    GeomMesh::Vertex* modelNormals = new GeomMesh::Vertex[faceCount * 3];

  	for (unsigned int i = 0; i < faceCount; i++) {
		PlyFace face = faces[i];

		//cout << "face " << i << " has " << face.nverts << " vertices" << endl;

		for (int j = 0; j < 3; j++) {
			int index = htonl(face.vertexIndices[j]);
			//cout << "index = " << index << endl;

			PlyVertex vertex = vertices[index];

			float x = vertex.x;
			float y = vertex.y;
			float z = vertex.z;

			float nx = vertex.nx;
			float ny = vertex.ny;
			float nz = vertex.nz;

			if (x > meshBBmax[0])
				meshBBmax.data[0] = x;
			else if (x < meshBBmin[0])
				meshBBmin.data[0] = x;

			if (y > meshBBmax[1])
				meshBBmax.data[1] = y;
			else if (y < meshBBmin[1])
				meshBBmin.data[1] = y;

			if (z > meshBBmax[2])
				meshBBmax.data[2] = z;
			else if (z < meshBBmin[2])
				meshBBmin.data[2] = z;

			modelVertices[i * 3 + j][0] = x;
			modelVertices[i * 3 + j][1] = y;
			modelVertices[i * 3 + j][2] = z;

			modelNormals[i * 3 + j][0] = nx;
			modelNormals[i * 3 + j][1] = ny;
			modelNormals[i * 3 + j][2] = nz;
		}
	}

  	GeomMesh* geomMesh = new GeomMesh(faceCount, modelVertices, modelNormals);
  	geomMesh->setBB(meshBBmin, meshBBmax);
	
	return geomMesh;
}
