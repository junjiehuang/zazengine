#include "FrameBufferObject.h"

#include "../ZazenGraphics.h"
#include "../Util/GLUtils.h"

#include <iostream>

#include <assert.h>

using namespace std;

FrameBufferObject*
FrameBufferObject::create()
{
	GLuint id = 0;

	// generate the id of our frame-buffer-object
	glGenFramebuffers( 1, &id );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::create: glGenFramebuffers failed - exit";
		return false;
	}

	return new FrameBufferObject( id );
}

bool
FrameBufferObject::destroy( FrameBufferObject* frameBufferObject )
{
	if ( NULL == frameBufferObject )
	{
		return true;
	}

	glDeleteFramebuffers( 1, &frameBufferObject->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	delete frameBufferObject;

	// NOTE: render-targets are owned by RenderTarget-class because matching depth/shadow targets are shared

	return true;
}

FrameBufferObject::FrameBufferObject( GLuint id )
{
	this->m_id = id;

	this->m_depthBuffer = 0;
	this->m_depthTarget = NULL;
}

FrameBufferObject::~FrameBufferObject()
{
	this->m_id = 0;

	this->m_depthBuffer = 0;
	this->m_depthTarget = NULL;
}

bool
FrameBufferObject::attachTarget( RenderTarget* renderTarget )
{
	// ignore when already added
	for ( unsigned int i = 0; i < this->m_attachedTargets.size(); i++ )
	{
		if ( renderTarget->getId() == this->m_attachedTargets[ i ]->getId() )
		{
			return true;
		}
	}

	// any depth-target is regarded as depth-buffer
	if ( RenderTarget::RT_DEPTH == renderTarget->getType() || RenderTarget::RT_DEPTH_STENCIL == renderTarget->getType() ||
		 RenderTarget::RT_SHADOW_PLANAR == renderTarget->getType() || RenderTarget::RT_SHADOW_CUBE == renderTarget->getType() )
	{
		this->m_depthBuffer = renderTarget->getId();
		this->m_depthTarget = renderTarget;
	
		if ( false == this->attachDepthTargetTemp( renderTarget ) )
		{
			return false;
		}

	}
	else if ( RenderTarget::RT_COLOR == renderTarget->getType() )
	{
		GLuint id = renderTarget->getId();
		GLenum colorAttachment = GL_COLOR_ATTACHMENT0 + this->m_colorBufferTargets.size();

		if ( false == this->attachColorTargetTemp( renderTarget, this->m_colorBufferTargets.size() ) )
		{
			return false;
		}

		this->m_colorBufferTargets.push_back( colorAttachment );
	}

	this->m_attachedTargets.push_back( renderTarget );

	return true;
}

bool
FrameBufferObject::attachDepthTargetTemp( RenderTarget* renderTarget )
{
	if ( RenderTarget::RT_DEPTH == renderTarget->getType() || RenderTarget::RT_SHADOW_PLANAR == renderTarget->getType() || RenderTarget::RT_SHADOW_CUBE == renderTarget->getType() )
	{
		// add this as a depth-attachment to get correct depth-visibility in our deferred rendering
		glFramebufferTexture( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, renderTarget->getId(), 0 );
		GL_PEEK_ERRORS_AT_DEBUG
	}
	else if ( RenderTarget::RT_DEPTH_STENCIL == renderTarget->getType() )
	{
		// add this as a depth- AND stencil-attachment to get correct depth-visibility and stencil-buffer in our deferred rendering
		glFramebufferTexture( GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, renderTarget->getId(), 0 );
		GL_PEEK_ERRORS_AT_DEBUG
	}
	
	return true;
}

bool
FrameBufferObject::attachColorTargetTemp( RenderTarget* renderTarget, unsigned int colorAttachmentIndex )
{
	if ( RenderTarget::RT_COLOR == renderTarget->getType() )
	{
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + colorAttachmentIndex, GL_TEXTURE_2D, renderTarget->getId(), 0 );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	return true;
}

bool
FrameBufferObject::detachColorTargetTemp( RenderTarget* renderTarget, unsigned int colorAttachmentIndex )
{
	glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + colorAttachmentIndex, GL_TEXTURE_2D, 0, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::attachDepthTargetTempCubeFace( RenderTarget* renderTarget, unsigned int face )
{
	if ( RenderTarget::RT_SHADOW_CUBE == renderTarget->getType() )
	{
		// add this as a depth-attachment to get correct depth-visibility in our deferred rendering
		// for a shadow cube-map we need to attach all 6 faces separately
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, renderTarget->getId(), 0 );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	return true;
}

bool
FrameBufferObject::attachColorTargetTempCubeFace( RenderTarget* renderTarget, unsigned int face, unsigned int colorAttachmentIndex )
{
	if ( RenderTarget::RT_COLOR_CUBE == renderTarget->getType() )
	{
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + colorAttachmentIndex, GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, renderTarget->getId(), 0 );
		GL_PEEK_ERRORS_AT_DEBUG
	}

	return true;
}

bool
FrameBufferObject::restoreDepthTarget()
{
	if ( this->m_depthTarget )
	{
		return this->attachDepthTargetTemp( this->m_depthTarget );
	}

	return false;
}

bool
FrameBufferObject::restoreColorTarget( unsigned int targetIndex )
{
	RenderTarget* colorTarget = this->m_attachedTargets[ targetIndex ];

	return this->attachColorTargetTemp( colorTarget, targetIndex );
}

bool
FrameBufferObject::bind()
{
	// TODO: implement a guard: if FBO already bound, don't call api

	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::blitColorToSystemFB( unsigned int targetIndex )
{
	RenderTarget* target = this->m_attachedTargets[ targetIndex ];

	// specify this FBO as the read-fbo
	glBindFramebuffer( GL_READ_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the read-target
	glReadBuffer( this->m_colorBufferTargets[ targetIndex ] );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify the system framebuffer as draw-fbo
	glBindFramebuffer( GL_DRAW_FRAMEBUFFER, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	// perform the blit-operation
	glBlitFramebuffer( 0, 0, target->getWidth(), target->getHeight(), 0, 0, target->getWidth(), target->getHeight(), GL_COLOR_BUFFER_BIT, GL_NEAREST );
	GL_PEEK_ERRORS_AT_DEBUG
		
	// unbind this FBO as read-target
	glBindFramebuffer( GL_READ_FRAMEBUFFER, 0 );
	GL_PEEK_ERRORS_AT_DEBUG 

	return true;
}

bool
FrameBufferObject::blitColorToFBO( RenderTarget* target, unsigned int srcIndex, unsigned int dstIndex, FrameBufferObject* targetFbo )
{
	// specify this FBO as the read-fbo
	glBindFramebuffer( GL_READ_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the read-target
	glReadBuffer( GL_COLOR_ATTACHMENT0 + srcIndex);
	GL_PEEK_ERRORS_AT_DEBUG

	CHECK_FRAMEBUFFER_DEBUG

	// specify the system framebuffer as draw-fbo
	glBindFramebuffer( GL_DRAW_FRAMEBUFFER, targetFbo->getId() );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the draw-target
	glDrawBuffer( GL_COLOR_ATTACHMENT0 + dstIndex );
	GL_PEEK_ERRORS_AT_DEBUG

	CHECK_FRAMEBUFFER_DEBUG

	// perform the blit-operation
	glBlitFramebuffer( 0, 0, target->getWidth(), target->getHeight(), 0, 0, target->getWidth(), target->getHeight(), GL_COLOR_BUFFER_BIT, GL_NEAREST );
	GL_PEEK_ERRORS_AT_DEBUG
		
	// bind this FBO again as read&draw framebuffer (as it was before)
	glBindFramebuffer( GL_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG 

	return true;
}


bool
FrameBufferObject::blitDepthToFBO( FrameBufferObject* targetFbo )
{
	RenderTarget* target = this->m_depthTarget;

	// specify this FBO as the read-fbo
	glBindFramebuffer( GL_READ_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the read-target
	glReadBuffer( GL_NONE );
	GL_PEEK_ERRORS_AT_DEBUG

	CHECK_FRAMEBUFFER_DEBUG
	
	// specify the system framebuffer as draw-fbo
	glBindFramebuffer( GL_DRAW_FRAMEBUFFER, targetFbo->getId() );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the read-target
	glDrawBuffer( GL_NONE );
	GL_PEEK_ERRORS_AT_DEBUG

	CHECK_FRAMEBUFFER_DEBUG

	// perform the blit-operation
	glBlitFramebuffer( 0, 0, target->getWidth(), target->getHeight(), 0, 0, target->getWidth(), target->getHeight(), GL_DEPTH_BUFFER_BIT, GL_NEAREST );
	GL_PEEK_ERRORS_AT_DEBUG

	// bind this FBO again as read&draw framebuffer (as it was before)
	glBindFramebuffer( GL_FRAMEBUFFER, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG 

	return true;
}

bool
FrameBufferObject::blitColorFromTo( unsigned int srcTargetIndex, unsigned int dstTargetIndex )
{
	RenderTarget* srcTarget = this->m_attachedTargets[ srcTargetIndex ];
	RenderTarget* dstTarget = this->m_attachedTargets[ dstTargetIndex ];

	glReadBuffer( this->m_colorBufferTargets[ srcTargetIndex ] );
	GL_PEEK_ERRORS_AT_DEBUG

	// specify this render-target as the read-target
	glDrawBuffer( this->m_colorBufferTargets[ dstTargetIndex ] );
	GL_PEEK_ERRORS_AT_DEBUG

	// perform the blit-operation
	glBlitFramebuffer( 0, 0, srcTarget->getWidth(), srcTarget->getHeight(), 0, 0, dstTarget->getWidth(), dstTarget->getHeight(), GL_COLOR_BUFFER_BIT, GL_NEAREST );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::unbind()
{
	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::bindAllTargets()
{
	// bind the mrts
	for ( unsigned int i = 0; i < this->m_attachedTargets.size(); i++ )
	{
		if ( false == this->m_attachedTargets[ i ]->bind( i ) )
		{
			return false;
		}
	}

	return true;
}

bool
FrameBufferObject::bindTargets( std::vector<unsigned int> indices )
{
	// bind the mrts
	for ( unsigned int i = 0; i < indices.size(); i++ )
	{
		if ( false == this->m_attachedTargets[ indices[ i ] ]->bind( i ) )
		{
			return false;
		}
	}

	return true;
}

bool
FrameBufferObject::bindTarget( unsigned int index )
{
	if ( false == this->m_attachedTargets[ index ]->bind( 0 ) )
	{
		return false;
	}

	return true;
}

bool
FrameBufferObject::drawAllBuffers()
{
	// activate multiple drawing to our color targets targets
	glDrawBuffers( this->m_colorBufferTargets.size(), &this->m_colorBufferTargets[ 0 ] );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::drawBuffers( std::vector<unsigned int> indices )
{
	std::vector<GLenum> colorBufferTargets;

	for ( unsigned int i = 0; i < indices.size(); i++ )
	{
		colorBufferTargets.push_back( this->m_colorBufferTargets[ indices[ i ] ] );
	}

	// activate multiple drawing to our color targets targets
	glDrawBuffers( colorBufferTargets.size(), &colorBufferTargets[ 0 ] );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::drawBuffer( unsigned int index )
{
	// activate multiple drawing to our color targets targets
	glDrawBuffer( this->m_colorBufferTargets[ index ] );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::drawNone()
{
	glDrawBuffer( GL_NONE );
	GL_PEEK_ERRORS_AT_DEBUG
	glReadBuffer( GL_NONE );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::clearAll()
{
	if ( false == this->drawAllBuffers() )
	{
		return false;
	}

	// turn on color drawing ( was turned off in shadowmaping )
	// clear the colorbuffers AND our depth-buffer ( m_geometryDepth );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
FrameBufferObject::checkStatus()
{
	GLenum status = glCheckFramebufferStatus( GL_FRAMEBUFFER );

	if ( GL_FRAMEBUFFER_COMPLETE == status )
	{
		return true;
	}
	else if ( GL_FRAMEBUFFER_UNSUPPORTED == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_UNSUPPORTED" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" );
	}
	else if ( GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" );
	}
	else if ( GL_FRAMEBUFFER_BINDING == status )
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" );
	}
	else
	{
		ZazenGraphics::getInstance().getLogger().logError( "FrameBufferObject::checkStatus: unknown status " + status );
	}

	return false;
}
