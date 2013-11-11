#ifndef _GEOMETRY_FACTORY_H_
#define _GEOMETRY_FACTORY_H_

#include "MeshNode.h"
#include "MeshStatic.h"
#include "MeshBoned.h"

#include <assimp/scene.h>

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class GeometryFactory
{
	public:
		static void init( const boost::filesystem::path& );
		static void freeAll();
		static GeometryFactory& getRef() { return *GeometryFactory::instance; };

		MeshNode* getMesh( const std::string& fileName );
		MeshStatic* createQuad( float width, float height );

		void free( Mesh* );

	private:
		struct MeshBone {
			std::string m_name;
			glm::mat4 m_offset; // mesh to bone
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

		MeshBoned* processMeshBoned( const struct aiMesh* );
		MeshStatic* processMeshStatic( const struct aiMesh* );
		
		int getBoneIndex( const std::string& );

		static void processBoneWeight( MeshBoned::BonedVertexData&, const aiVertexWeight&, unsigned int );
		static void updateBB( const aiVector3D& vertex, glm::vec3&, glm::vec3& );
};

#endif /* _GEOMETRY_FACTORY_H_ */
