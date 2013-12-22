#ifndef _GEOMETRY_FACTORY_H_
#define _GEOMETRY_FACTORY_H_

#include "MeshNode.h"

#include <assimp/scene.h>

#include <boost/filesystem.hpp>

#include <GL/glew.h>

#include <map>
#include <string>

class GeometryFactory
{
	public:
		static void init( const boost::filesystem::path& );
		static void freeAll();
		static GeometryFactory& getRef() { return *GeometryFactory::instance; };

		MeshNode* getMesh( const std::string& fileName );
		Mesh* createQuad( float width, float height );
		Mesh* createUnitCube();
		Mesh* createSphere( float radius, unsigned int rings, unsigned int sectors );

		void free( Mesh* );

	private:
		typedef float Vertex [ 3 ];
		typedef float Normal [ 3 ];
		typedef float TexCoord [ 2 ];
		typedef float Tangent [ 3 ];
		typedef unsigned int BoneIndex[ 4 ];
		typedef float BoneWeight[ 4 ];

		struct MeshBone {
			std::string m_name;
			glm::mat4 m_offset; // mesh to bone
		};

		struct BonedVertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
			unsigned int boneCount;
			BoneIndex boneIndices;
			BoneWeight boneWeights;
		};

		struct StaticVertexData
		{
			Vertex position;
			Normal normal;
			TexCoord texCoord;
			Tangent tangent;
		};

		struct QuadVertexData
		{
			Vertex position;
			TexCoord texCoord;
		};

		static GeometryFactory* instance;

		boost::filesystem::path m_modelDataPath;
		std::map<std::string, MeshNode*> m_allMeshes;

		const struct aiScene* m_currentScene;
		std::vector<MeshBone> m_currentBonesHierarchical;

		GeometryFactory( const boost::filesystem::path& );
		~GeometryFactory();

		MeshNode* loadFolder( const boost::filesystem::path& );
		MeshNode* loadFile( const boost::filesystem::path& );

		void collectBonesHierarchical( const struct aiNode* );

		MeshNode* processNode( const struct aiNode*, const glm::mat4& );		 
		Mesh* processMesh( const struct aiMesh* );

		Mesh* processMeshBoned( const struct aiMesh* );
		Mesh* processMeshStatic( const struct aiMesh* );
		
		int getBoneIndex( const std::string& );

		static void processBoneWeight( BonedVertexData&, const aiVertexWeight&, unsigned int );
		static void updateBB( const aiVector3D& vertex, glm::vec3&, glm::vec3& );
};

#endif /* _GEOMETRY_FACTORY_H_ */
