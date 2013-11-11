#include "Animation.h"

#include "../ZazenGraphics.h"

using namespace std;

// this is a copy of the slerp-method of GLM because glm 0.9.4.6 has a bug complaining about 2 overloaded definitions
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
	this->m_animationChannels = NULL;

	this->m_currentTime = 0.0;
	this->m_currentFrame = 0.0;
	
	this->m_lastPosChannelIndex = 0;
	this->m_lastRotChannelIndex = 0;
	this->m_lastScaleChannelIndex = 0;

	// default to 25 ticks/second if 0
	this->m_ticksPerSecond = ticksPerSecond == 0.0 ? 25.0 : ticksPerSecond;
	this->m_durationTicks = durationTicks;
	this->m_durationInSec = this->m_durationTicks / this->m_ticksPerSecond;
}

Animation::~Animation()
{
	if ( this->m_skeletonRoot )
	{
		this->cleanUpSkeletonPart( this->m_skeletonRoot );
		this->m_skeletonRoot = NULL;
	}

	// NOTE: don't delete m_animationChannels because IT IS OWNED BY AnimationFactory
}

void
Animation::update()
{
	// not initialized with a skeleton yet => exit, nothing to animate
	if ( NULL == this->m_skeletonRoot )
	{
		return;
	}

	double timeAdvance = ZazenGraphics::getInstance().getCore().getProcessingFactor();

	// reset bone-transformations
	this->m_boneTransforms.clear();

	// calculate advancement of animation-time: do modul by total duration of animation (between 0.0 and m_durationInSec)
	this->m_currentTime = fmod( ( this->m_currentTime + timeAdvance ), this->m_durationInSec );
	// calculate new frame-time: is not a whole-number but can be in-between of key-frames => we will do interpolation of key-frames for smooth animation
	// will be between 0.0 and m_durationTicks
	this->m_currentFrame = this->m_currentTime * this->m_ticksPerSecond;

	// do animation for this new frame-time
	this->animateSkeleton( this->m_skeletonRoot, glm::mat4( ) );
}

void
Animation::cleanUpSkeletonPart( AnimationSkeletonPart* skeletonPart )
{
	// all dynamic-memory stuff isn't owned by the AnimationSkeletonPart but just pointer-copy of MeshNode ( m_boneOffset ) and AnimationNode ( m_animationChannel )
	// except children: delete them all
	for ( unsigned int i = 0; i < skeletonPart->m_children.size(); i++ )
	{
		this->cleanUpSkeletonPart( skeletonPart->m_children[ i ] );
	}

	delete skeletonPart;
}

void
Animation::initSkeleton( MeshNode* rootMeshNode )
{
	// already initialized with a preceeding skeleton, clean this up first
	if ( this->m_skeletonRoot )
	{
		this->cleanUpSkeletonPart( this->m_skeletonRoot );
		this->m_skeletonRoot = NULL;
	}

	// mesh-node hierarchy specified => build a new skeleton
	if ( rootMeshNode )
	{
		this->m_skeletonRoot = this->buildAnimationSkeleton( rootMeshNode );
	}
}

// builds the hierarchical structure (animation-skeleton) based upon the Mesh-Node hierarchy
// this allows to separate mesh-loading and animation-loading and thus separation of meshes from animations
// NOTE: for this to work it is expected that compatible animations have at least all the bones found in the mesh-node hierarchy
Animation::AnimationSkeletonPart*
Animation::buildAnimationSkeleton( MeshNode* rootMeshNode )
{
	AnimationSkeletonPart* skeletonPart = new AnimationSkeletonPart();
	skeletonPart->m_animationChannel = NULL;
	skeletonPart->m_boneOffset = NULL;
	skeletonPart->m_localTransform = rootMeshNode->getLocalTransform();

	std::map<std::string, AnimationChannel>::iterator findAnimChannelsIter = this->m_animationChannels->find( rootMeshNode->getName() );

	// animation-channel is found for this name thus this node is present in the animation and thus is animated => copy pointer to animation-channels
	if ( this->m_animationChannels->end() != findAnimChannelsIter )
	{
		skeletonPart->m_animationChannel = &findAnimChannelsIter->second;

		// this mesh-node has a bone => store offset (mesh-to-bone transform) in according skeleton-part
		if (rootMeshNode->getBoneOffset() )
		{
			skeletonPart->m_boneOffset = rootMeshNode->getBoneOffset();
		}
	}

	// walk down recursively
	const std::vector<MeshNode*>& children = rootMeshNode->getChildren();
	for ( unsigned int i = 0; i < children.size(); i++ )
	{
		AnimationSkeletonPart* childSkeletonPart = this->buildAnimationSkeleton( children[ i ] );
		skeletonPart->m_children.push_back( childSkeletonPart );
	}

	return skeletonPart;
}

// performs hierarchical animation by walking the animation-skeleton hierarchical
void
Animation::animateSkeleton( AnimationSkeletonPart* skeletonPart, const glm::mat4& parentGlobalTransform )
{
	glm::mat4 localNodeTransform;

	// check if this node has animation-channels stored, if so => interpolate key-frames
	if ( skeletonPart->m_animationChannel )
	{
		glm::mat4 translationMatrix;
		glm::mat4 rotationMatrix;
		glm::mat4 scalingMatrix;

		this->interpolatePosition( skeletonPart->m_animationChannel->m_positionKeys, translationMatrix );
		this->interpolateRotation( skeletonPart->m_animationChannel->m_rotationKeys, rotationMatrix );
		this->interpolateScaling( skeletonPart->m_animationChannel->m_scalingKeys, translationMatrix );

		// this animation-transformation replaces the local transformation
		// order is important: first scaling, then rotation and last translation (matrix-multiplication applies the operation first which one was multiplied with last)
		localNodeTransform = translationMatrix * rotationMatrix * scalingMatrix;
	}
	else
	{
		// no animation for this node: use local transformation already present
		localNodeTransform = skeletonPart->m_localTransform;
	}

	// do hierarchical transformation: apply parent-transformations to local Transformation
	glm::mat4 globalTransform = parentGlobalTransform * localNodeTransform;

	// check if this node has a bone assosiated with
	if ( skeletonPart->m_boneOffset )
	{
		// calculate bone-transformation: first apply m_offset matrix which transposes the vertex back into bone-relative
		// space which can be thought of the bone at the center of the coordinate space and the vertex now relative to 
		// the bone. then the hierarchical node transformation is applied which will move the vertex to its right position
		// within the animation hierarchy
		glm::mat4 globalBoneTransform = globalTransform * ( * (skeletonPart->m_boneOffset ) );
		
		// add this bone to the master-list of the final bone transformations
		// NOTE: the order of this bone-list must match EXACTLY the bone-list extracted in GeometryFactory
		// otherwise we would wind up with wrong bone-indices which would lead to wrong renderings of the mesh
		this->m_boneTransforms.push_back( globalBoneTransform );
	}

	// recursively walk down tree
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

	// find key-frames for the current-frame
	this->findFirstAndNext<glm::vec3>( positionKeys, firstPos, nextPos, this->m_lastPosChannelIndex );

	// calculate interpolation-factor (between 0.0 and 1.0)
	float interpolationFactor = ( float ) ( ( this->m_currentFrame - firstPos.m_time ) / ( nextPos.m_time - firstPos.m_time ) );

	// do linear interpolation of vectors using glm::lerp
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

	// find key-frames for the current-frame
	this->findFirstAndNext<glm::quat>( rotationKeys, firstRot, nextRot, this->m_lastRotChannelIndex );

	// calculate interpolation-factor (between 0.0 and 1.0)
	float interpolationFactor = ( float ) ( ( this->m_currentFrame - firstRot.m_time ) / ( nextRot.m_time - firstRot.m_time ) );

	// do spherical interpolation of quaternions
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
	
	// find key-frames for the current-frame
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

	// find the two key-frames for which the currentFrame lies in between those two
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
