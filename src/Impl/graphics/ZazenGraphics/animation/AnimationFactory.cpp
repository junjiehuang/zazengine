#include "AnimationFactory.h"

#include "../ZazenGraphics.h"

#include "../Util/AssImpUtils.h"

#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

map<string, Animation*> AnimationFactory::allAnimations;
boost::filesystem::path AnimationFactory::animationDataPath;

void
AnimationFactory::setDataPath( const boost::filesystem::path& animationDataPath )
{
	AnimationFactory::animationDataPath = animationDataPath;
}

Animation*
AnimationFactory::get( const std::string& fileName )
{
	map<std::string, Animation*>::iterator findIter = AnimationFactory::allAnimations.find( fileName );
	if ( findIter != AnimationFactory::allAnimations.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( AnimationFactory::animationDataPath.generic_string() + fileName );
	if ( ! filesystem::exists( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::get: file \"" << fullFileName << "\" does not exist";
		return 0;	
	}

	if ( filesystem::is_directory( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::get: file \"" << fullFileName << "\" is a directory";
		return 0;
	}
	
	Animation* animation = AnimationFactory::loadFile( fullFileName );

	if ( NULL != animation )
	{
		AnimationFactory::allAnimations[ fileName ] = animation;
	}

	return animation;
}

vector<Animation*>
AnimationFactory::loadDir( const std::string& directory, const std::string& extension )
{
	vector<Animation*> rootAnimations;

	filesystem::path folderPath( AnimationFactory::animationDataPath.generic_string() + directory );
	if ( ! filesystem::exists( folderPath ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadDir: directory \"" << directory << "\" does not exist";
		return rootAnimations;	
	}

	filesystem::directory_iterator iter( folderPath );
	filesystem::directory_iterator endIter;

	// iterate through directory
	while ( iter != endIter )
	{
		filesystem::directory_entry entry = *iter++;
		
		// recursively go down into sub-directories
		if ( filesystem::is_directory( entry.path() ) )
		{
			vector<Animation*> subDirAnimations = AnimationFactory::loadDir( entry.path().generic_string().c_str(), extension );
			rootAnimations.insert( rootAnimations.begin(), subDirAnimations.begin(), subDirAnimations.end() ); 
		}
		else
		{
			// check if extension is the one we want
			string pathExtension = filesystem::extension( entry.path() );
			if ( pathExtension == extension )
			{
				// check if this file is already loaded
				map<std::string, Animation*>::iterator findIter = AnimationFactory::allAnimations.find( entry.path().generic_string().c_str() );
				if ( findIter != AnimationFactory::allAnimations.end() )
				{
					rootAnimations.push_back( findIter->second );
				}
				// not yet loaded
				else
				{
					// load animation
					Animation* animation = AnimationFactory::loadFile( entry.path().generic_string().c_str() );
					if ( 0 != animation )
					{
						rootAnimations.push_back( animation );
					}
				}
			}
		}
	}

	return rootAnimations;
}

void
AnimationFactory::freeAll()
{
	map<std::string, Animation*>::iterator iter = AnimationFactory::allAnimations.begin();
	while ( iter != AnimationFactory::allAnimations.end() )
	{
		Animation* anim = iter->second;
		delete anim;

		iter++;
	}

	AnimationFactory::allAnimations.clear();
}

Animation*
AnimationFactory::loadFile( const filesystem::path& filePath )
{
	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADING ... " << filePath;

	const std::string& fileName = filePath.generic_string();

	const struct aiScene* scene = aiImportFile( fileName.c_str(), 0 );
	if ( NULL == scene )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString();
		return 0;
	}

	if ( ! scene->HasAnimations() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: tried to load animation-file without animations - ignoring this file.";
		
		aiReleaseImport( scene );
		return 0;
	}

	Animation* animation = NULL;
	aiNode* rootNode = scene->mRootNode;

	// NOTE: for now only first animation is considered, should be enough for doom3
	aiAnimation* assImpAnim = scene->mAnimations[ 0 ];

	animation = new Animation( assImpAnim->mDuration, assImpAnim->mTicksPerSecond );

	// walk through all nodes in hierarchical manner (recursive) and store them in the animation-object and in the node-to-name map
	AnimationFactory::collectNodes( animation, rootNode );

	// walk through all bones in a hierarchical manner (recursive) and match them to their according AnimNode by matching names
	AnimationFactory::extractBones( animation, scene, rootNode );

	// extract data from all channels
	for ( unsigned int i = 0; i < assImpAnim->mNumChannels; i++ )
	{
		aiNodeAnim* assImpChannel = assImpAnim->mChannels[ i ];
			
		// each channel is assigned to exactly one node: find the node
		map<string, Animation::AnimationNode*>::iterator findIter = animation->m_animationNodes.find( assImpChannel->mNodeName.C_Str() );
		if ( animation->m_animationNodes.end() == findIter )
		{
			ZazenGraphics::getInstance().getLogger().logWarning() << "couldn't find an animation node for node " << assImpChannel->mNodeName.C_Str();
			continue;
		}
			
		Animation::AnimationChannel& animChannels = findIter->second->m_animationChannels;

		// extract all position-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumPositionKeys; j++ )
		{
			aiVectorKey assImpPosKey = assImpChannel->mPositionKeys[ j ];

			Animation::AnimationKey<glm::vec3> posKey;
			posKey.m_time = assImpPosKey.mTime;
	
			AssImpUtils::assimpVecToGlm( assImpPosKey.mValue, posKey.m_value );

			animChannels.m_positionKeys.push_back( posKey );
		}

		// extract all rotation-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumRotationKeys; j++ )
		{
			aiQuatKey assImpRotKey = assImpChannel->mRotationKeys[ j ];

			Animation::AnimationKey<glm::quat> rotKey;
			rotKey.m_time = assImpRotKey.mTime;
	
			// assimp provides the rotations by quaternions, get matrix from it
			AssImpUtils::assimpQuatToGlm( assImpRotKey.mValue, rotKey.m_value );

			animChannels.m_rotationKeys.push_back( rotKey );
		}

		// extract all scaling-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumScalingKeys; j++ )
		{
			aiVectorKey assImpScaleKey = assImpChannel->mScalingKeys[ j ];

			Animation::AnimationKey<glm::vec3> scaleKey;
			scaleKey.m_time = assImpScaleKey.mTime;
	
			AssImpUtils::assimpVecToGlm( assImpScaleKey.mValue, scaleKey.m_value );

			animChannels.m_scalingKeys.push_back( scaleKey );
		}
	}
		
	AnimationFactory::allAnimations[ filePath.generic_string().c_str() ] = animation; 

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;

    return animation;
}

Animation::AnimationNode*
AnimationFactory::collectNodes( Animation* animation, const aiNode* assImpNode )
{
	Animation::AnimationNode* parentNode = new Animation::AnimationNode();

	parentNode->m_name = assImpNode->mName.C_Str();
	AssImpUtils::assimpMatToGlm( assImpNode->mTransformation, parentNode->m_transform );

	animation->m_animationNodes[ parentNode->m_name ] = parentNode;

	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		aiNode* assImpChild = assImpNode->mChildren[ i ];

		Animation::AnimationNode* childNode = AnimationFactory::collectNodes( animation, assImpChild );
	}

	return parentNode;
}

void
AnimationFactory::extractBones( Animation* animation, const aiScene* assImpScene, const aiNode* assImpNode )
{
	for ( unsigned int i = 0; i < assImpNode->mNumMeshes; i++ )
	{
		aiMesh* mesh = assImpScene->mMeshes[ i ];
			
		for ( unsigned int j = 0; j < mesh->mNumBones; j++ )
		{
			aiBone* assImpBone = mesh->mBones[ j ];
			
			// ignore bones with no name
			if ( 0 == assImpBone->mName.length )
			{
				continue;
			}

			Animation::AnimationBone* bone = new Animation::AnimationBone();
			bone->m_name = assImpBone->mName.C_Str();
			AssImpUtils::assimpMatToGlm( assImpBone->mOffsetMatrix, bone->m_meshToBoneTransf );

			animation->m_animationBones[ bone->m_name ] = bone;
		}
	}

	// recursively extract bones from children
	for ( unsigned int i = 0; i < assImpNode->mNumChildren; i++ )
	{
		aiNode* child = assImpNode->mChildren[ i ];
		AnimationFactory::extractBones( animation, assImpScene, child );
	}
}
