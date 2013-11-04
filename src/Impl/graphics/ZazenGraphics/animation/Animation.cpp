#include "Animation.h"

#include "../ZazenGraphics.h"

using namespace std;

Animation::Animation( double durationTicks, double ticksPerSecond )
{
	this->m_rootNode = NULL;

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
Animation::perform( double timeAdvance )
{
	this->m_transforms.clear();

	this->m_currentTime = fmod( ( this->m_currentTime + timeAdvance ), this->m_durationInSec );
	this->m_currentFrame = this->m_currentTime * this->m_ticksPerSecond;

	this->performRecursive( this->m_rootNode, this->m_rootNode->getTransform() );
}

void
Animation::updateToProgram( Program* program )
{
	program->setUniformMatrices( "u_bones[0]", this->m_transforms );
}

void
Animation::performRecursive( AnimNode* node, const glm::mat4& parentTransform )
{
	glm::mat4 nodeTransform;

	if ( node->getChannel() )
	{
		const AnimNode::AnimChannel* channel = node->getChannel();

		glm::mat4 translationMatrix;
		glm::mat4 rotationMatrix;
		glm::mat4 scalingMatrix;

		this->interpolatePosition( channel->m_positionKeys, translationMatrix );
		this->interpolateRotation( channel->m_rotationKeys, rotationMatrix );
		this->interpolateScaling( channel->m_scalingKeys, translationMatrix );

		// order is important: first scaling, then rotation and last translation (matrix-multiplication applies the operation first which one was multiplied with last)
		nodeTransform = translationMatrix * rotationMatrix * scalingMatrix;
	}

	// apply hierarchical transformation
	nodeTransform = parentTransform * nodeTransform;

	if ( node->getBone() )
	{
		const AnimNode::Bone* bone = node->getBone();
		nodeTransform = nodeTransform * bone->m_meshToBoneTransf;

		//ZazenGraphics::getInstance().getLogger().logWarning() << "bone \"" << bone->m_name << "\" has index of " << this->m_transforms.size();

		this->m_transforms.push_back( nodeTransform );
	}

	const std::vector<AnimNode*>& nodeChildren = node->getChildren();

	for ( unsigned int i = 0; i < nodeChildren.size(); i++ )
	{
		this->performRecursive( nodeChildren[ i ], nodeTransform );
	}
}

void
Animation::interpolatePosition( const vector<AnimNode::AnimKey<glm::vec3>>& positionKeys, glm::mat4& translationMatrix )
{
	// no translations, leave untouched
	if ( 0 == positionKeys.size() )
	{
		return;
	}

	// exactly one translation, nothing to interpolate
	if ( positionKeys.size() == 1 )
	{
		translationMatrix[ 3 ][ 0 ] = positionKeys[ 0 ].m_value.x;
		translationMatrix[ 3 ][ 1 ] = positionKeys[ 0 ].m_value.y;
		translationMatrix[ 3 ][ 2 ] = positionKeys[ 0 ].m_value.z;

		return;
	}

	AnimNode::AnimKey<glm::vec3> firstPos;
	AnimNode::AnimKey<glm::vec3> nextPos;

	this->findFirstAndNext<glm::vec3>( positionKeys, firstPos, nextPos, this->m_lastPosChannelIndex );

	float normalizedTime = ( float ) ( ( this->m_currentFrame - firstPos.m_time ) / ( nextPos.m_time - firstPos.m_time ) );

	glm::vec3 interpolatedPos = ( ( 1.0f - normalizedTime ) * firstPos.m_value ) + ( normalizedTime * nextPos.m_value );
	translationMatrix[ 3 ][ 0 ] = interpolatedPos.x;
	translationMatrix[ 3 ][ 1 ] = interpolatedPos.y;
	translationMatrix[ 3 ][ 2 ] = interpolatedPos.z;
}

void
Animation::interpolateRotation( const std::vector<AnimNode::AnimKey<glm::quat>>& rotationKeys, glm::mat4& rotationMatrix )
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

	AnimNode::AnimKey<glm::quat> firstRot;
	AnimNode::AnimKey<glm::quat> nextRot;

	this->findFirstAndNext<glm::quat>( rotationKeys, firstRot, nextRot, this->m_lastRotChannelIndex );

	// TODO: do interpolation of scaling between first and next
}

void
Animation::interpolateScaling( const std::vector<AnimNode::AnimKey<glm::vec3>>& scalingKeys, glm::mat4& scalingMatrix )
{
	// no scalings, leave untouched
	if ( 0 == scalingKeys.size() )
	{
		return;
	}

	// exactly one translation, nothing to interpolate
	if ( scalingKeys.size() == 1 )
	{
		scalingMatrix[ 3 ][ 0 ] = scalingKeys[ 0 ].m_value.x;
		scalingMatrix[ 3 ][ 1 ] = scalingKeys[ 0 ].m_value.y;
		scalingMatrix[ 3 ][ 2 ] = scalingKeys[ 0 ].m_value.z;

		return;
	}

	AnimNode::AnimKey<glm::vec3> firstScaling;
	AnimNode::AnimKey<glm::vec3> nextScaling;
	
	this->findFirstAndNext<glm::vec3>( scalingKeys, firstScaling, nextScaling, this->m_lastScaleChannelIndex );

	double deltaTime = nextScaling.m_time - firstScaling.m_time;

	// TODO: do interpolation of scaling between first and next
}

template <typename T>
void
Animation::findFirstAndNext( const vector<AnimNode::AnimKey<T>>& keys, AnimNode::AnimKey<T>& first, AnimNode::AnimKey<T>& next, unsigned int& lastChannelIndex )
{
	// NOTE: dirty-hack for doom3 models: we suppose that there are as many keys as maximum keys and all are equally spaced thus we can index right into the vector
	unsigned int index = ( unsigned int ) this->m_currentFrame;
	unsigned int nextIndex = ( index + 1 ) % keys.size();

	first = keys[ index ];
	next = keys[ nextIndex ];

	/*
	// NOTE: if the time would be equally spaced by 1 unit then m_time would match the index and we could access it in O(1)
	for ( unsigned int i = lastChannelIndex; i < keys.size(); i++ )
	{
		// TODO: something wrong here
		unsigned int nextIndex = ( i + 1 ) & ( keys.size() - 1 );
		// normal advance in indices
		if ( i < nextIndex )
		{
			if ( keys[ i ].m_time <= this->m_currentFrame && keys[ nextIndex ].m_time > this->m_currentFrame )
			{
				first = keys[ i ];
				next = keys[ nextIndex ];

				lastChannelIndex = i;
				return;
			}
		}
		// wrap-around
		else if ( i > nextIndex )
		{
			if ( keys[ i ].m_time <= this->m_currentFrame && keys[ nextIndex ].m_time < this->m_currentFrame )
			{
				first = keys[ i ];
				next = keys[ nextIndex ];

				lastChannelIndex = i;
				return;
			}
			else
			{
				// nothing found due to optimization (currentframe had a jump BEVORE lastChannelIndex): reset i to 0 and start from beginning
				if ( i == keys.size() - 1 )
				{
					i = 0;
				}
			}
		}
	}
	*/
}
