#include "Animation.h"

#include "../ZazenGraphics.h"

using namespace std;

// this is a copy of the slerp-method of GLM because glm 0.9.4.6 has a bug complaining about 2 overloaded defitions
glm::quat glmSlerpCopy( const glm::quat& x, const glm::quat& y, float a )
{
	glm::quat z = y;

	float cosTheta = glm::dot(x, y);

	// If cosTheta < 0, the interpolation will take the long way around the sphere. 
	// To fix this, one quat must be negated.
	if (cosTheta < 0.0f)
	{
		z        = -y;
		cosTheta = -cosTheta;
	}

	// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
	if(cosTheta > 1.0f - glm::epsilon<float>())
	{
		// Linear interpolation
		return glm::quat(
			glm::mix(x.w, z.w, a),
			glm::mix(x.x, z.x, a),
			glm::mix(x.y, z.y, a),
			glm::mix(x.z, z.z, a));
	}
	else
	{
		// Essential Mathematics, page 467
		float angle = acos(cosTheta);
		return (sin((1.0f - a) * angle) * x + sin(a * angle) * z) / sin(angle);
	}
}

Animation::Animation( double durationTicks, double ticksPerSecond )
{
	this->m_skeletonRoot = NULL;

	this->m_currentTime = 0.0;
	this->m_currentFrame = 0.0;
	
	this->m_lastPosChannelIndex = 0;
	this->m_lastRotChannelIndex = 0;
	this->m_lastScaleChannelIndex = 0;

	this->m_ticksPerSecond = ticksPerSecond;
	this->m_durationTicks = durationTicks;
	this->m_durationInSec = this->m_durationTicks / this->m_ticksPerSecond;
}

Animation::~Animation()
{
	// TODO: cleanup recursively
}

void
Animation::update()
{
	double timeAdvance = ZazenGraphics::getInstance().getCore().getProcessingFactor();

	// not initialized with a skeleton yet
	if ( NULL == this->m_skeletonRoot )
	{
		return;
	}

	this->m_transforms.clear();

	this->m_currentTime = fmod( ( this->m_currentTime + timeAdvance ), this->m_durationInSec );
	this->m_currentFrame = this->m_currentTime * this->m_ticksPerSecond;

	this->animateSkeleton( this->m_skeletonRoot, glm::mat4( ) );
}

void
Animation::initSkeleton( MeshNode* rootMeshNode )
{
	if ( NULL == this->m_skeletonRoot && rootMeshNode )
	{
		this->m_skeletonRoot = this->buildAnimationSkeleton( rootMeshNode, glm::mat4() );
	}
}

Animation::AnimationSkeletonPart*
Animation::buildAnimationSkeleton( MeshNode* rootMeshNode, const glm::mat4& parentGlobalTransform )
{
	AnimationSkeletonPart* skeletonPart = new AnimationSkeletonPart();
	skeletonPart->m_name = rootMeshNode->getName();
	skeletonPart->m_globalTransform = parentGlobalTransform * skeletonPart->m_localTransform;

	std::map<std::string, AnimationNode*>::iterator findAnimationNodeIter = this->m_animationNodes.find( rootMeshNode->getName() );
	std::map<std::string, AnimationBone*>::iterator findAnimationBoneIter = this->m_animationBones.find( rootMeshNode->getName() );

	if ( this->m_animationNodes.end() != findAnimationNodeIter )
	{
		skeletonPart->m_animationNode = findAnimationNodeIter->second;

		skeletonPart->m_localTransform = skeletonPart->m_animationNode->m_transform;
		skeletonPart->m_globalTransform = parentGlobalTransform * skeletonPart->m_localTransform;
	}

	if ( this->m_animationBones.end() != findAnimationBoneIter && rootMeshNode->getBone() )
	{
		skeletonPart->m_animationBone = findAnimationBoneIter->second;

		// bone is stored in row-major order => transpose it
		skeletonPart->m_animationBone->m_offset = glm::transpose( skeletonPart->m_animationBone->m_offset );
	}

	const std::vector<MeshNode*>& children = rootMeshNode->getChildren();
	for ( unsigned int i = 0; i < children.size(); i++ )
	{
		AnimationSkeletonPart* childSkeletonPart = this->buildAnimationSkeleton( children[ i ], skeletonPart->m_globalTransform );
		skeletonPart->m_children.push_back( childSkeletonPart );
	}

	return skeletonPart;
}

void
Animation::animateSkeleton( AnimationSkeletonPart* skeletonPart, const glm::mat4& parentGlobalTransform )
{
	glm::mat4 localNodeTransform;

	// check if this node has animation-information stored
	if ( skeletonPart->m_animationNode )
	{
		AnimationNode* animationNode = skeletonPart->m_animationNode;
		AnimationChannel& channel = animationNode->m_animationChannels;

		glm::mat4 translationMatrix;
		glm::mat4 rotationMatrix;
		glm::mat4 scalingMatrix;

		this->interpolatePosition( channel.m_positionKeys, translationMatrix );
		this->interpolateRotation( channel.m_rotationKeys, rotationMatrix );
		this->interpolateScaling( channel.m_scalingKeys, translationMatrix );

		// this animation-transformation replaces the local transformation
		// order is important: first scaling, then rotation and last translation (matrix-multiplication applies the operation first which one was multiplied with last)
		localNodeTransform = translationMatrix * rotationMatrix * scalingMatrix;
	}
	else
	{
		// no animation for this node: use local transformation 
		localNodeTransform = skeletonPart->m_localTransform;
	}

	// do hierarchical transformation: apply parentTransformation to local Transformation
	glm::mat4 globalTransform = parentGlobalTransform * localNodeTransform;

	// check if this node has a bone assosiated with
	if ( skeletonPart->m_animationBone )
	{
		// calculate bone-transformation: first apply m_offset matrix which transposes the vertex back into bone-relative
		// space which can be thought of the bone at the center of the coordinate space and the vertex now relative to 
		// the bone. then the hierarchical node transformation is applied which will move the vertex to its right position
		// within the animation hierarchy
		glm::mat4 globalBoneTransform = globalTransform * skeletonPart->m_animationBone->m_offset;

		this->m_transforms.push_back( globalBoneTransform );
	}

	const std::vector<AnimationSkeletonPart*>& skeletonChildren = skeletonPart->m_children;

	for ( unsigned int i = 0; i < skeletonChildren.size(); i++ )
	{
		this->animateSkeleton( skeletonChildren[ i ], globalTransform );
	}
}

void
Animation::interpolatePosition( const vector<AnimationKey<glm::vec3>>& positionKeys, glm::mat4& translationMatrix )
{
	// no translations, leave untouched
	if ( 0 == positionKeys.size() )
	{
		return;
	}

	// exactly one translation, nothing to interpolate
	if ( positionKeys.size() == 1 )
	{
		translationMatrix = glm::translate( positionKeys[ 0 ].m_value );

		return;
	}

	AnimationKey<glm::vec3> firstPos;
	AnimationKey<glm::vec3> nextPos;

	this->findFirstAndNext<glm::vec3>( positionKeys, firstPos, nextPos, this->m_lastPosChannelIndex );

	float interpolationFactor = ( float ) ( ( this->m_currentFrame - firstPos.m_time ) / ( nextPos.m_time - firstPos.m_time ) );

	translationMatrix = glm::translate( glm::lerp( firstPos.m_value, nextPos.m_value, interpolationFactor ) );
}

void
Animation::interpolateRotation( const std::vector<AnimationKey<glm::quat>>& rotationKeys, glm::mat4& rotationMatrix )
{
	// no rotations, leave untouched
	if ( 0 == rotationKeys.size() )
	{
		return;
	}

	// exactly one rotation, nothing to interpolate
	if ( rotationKeys.size() == 1 )
	{
		rotationMatrix = glm::mat4_cast( rotationKeys[ 0 ].m_value );

		return;
	}

	AnimationKey<glm::quat> firstRot;
	AnimationKey<glm::quat> nextRot;

	this->findFirstAndNext<glm::quat>( rotationKeys, firstRot, nextRot, this->m_lastRotChannelIndex );

	float interpolationFactor = ( float ) ( ( this->m_currentFrame - firstRot.m_time ) / ( nextRot.m_time - firstRot.m_time ) );

	//rotationMatrix = glm::mat4_cast( glm::slerp( firstRot.m_value, nextRot.m_value, interpolationFactor ) );

	rotationMatrix = glm::mat4_cast( glmSlerpCopy( firstRot.m_value, nextRot.m_value, interpolationFactor ) );
}

void
Animation::interpolateScaling( const std::vector<AnimationKey<glm::vec3>>& scalingKeys, glm::mat4& scalingMatrix )
{
	// no scalings, leave untouched
	if ( 0 == scalingKeys.size() )
	{
		return;
	}

	// exactly one translation, nothing to interpolate
	if ( scalingKeys.size() == 1 )
	{
		scalingMatrix[ 0 ].x = scalingKeys[ 0 ].m_value.x;
		scalingMatrix[ 1 ].y = scalingKeys[ 0 ].m_value.y;
		scalingMatrix[ 2 ].z = scalingKeys[ 0 ].m_value.z;

		return;
	}

	AnimationKey<glm::vec3> firstScaling;
	AnimationKey<glm::vec3> nextScaling;
	
	this->findFirstAndNext<glm::vec3>( scalingKeys, firstScaling, nextScaling, this->m_lastScaleChannelIndex );

	// for now: now interpolation, just take first found
	scalingMatrix[ 0 ].x = firstScaling.m_value.x;
	scalingMatrix[ 1 ].y = firstScaling.m_value.y;
	scalingMatrix[ 2 ].z = firstScaling.m_value.z;

	// TODO: do interpolation of scaling between first and next
}

template <typename T>
void
Animation::findFirstAndNext( const vector<AnimationKey<T>>& keys, AnimationKey<T>& first, AnimationKey<T>& next, unsigned int& lastChannelIndex )
{
	// jump in time or frame turned around => reset last index to 0
	if ( ( unsigned int ) this->m_currentFrame < lastChannelIndex - 1 )
	{
		lastChannelIndex = 0;
	}

	for ( unsigned int i = lastChannelIndex; i < keys.size() - 1; i++ )
	{
		first = keys[ i ];
		next = keys[ i + 1 ];
	
		// does always hit or if not we reached end of loop
		if ( first.m_time <= this->m_currentFrame && next.m_time > this->m_currentFrame )
		{
			lastChannelIndex = i;
			return;
		}
	}
}
