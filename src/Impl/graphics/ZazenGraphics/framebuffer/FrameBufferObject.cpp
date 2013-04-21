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
	if ( false == GLUtils::peekErrors() )
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

	for ( unsigned int i = 0; i < frameBufferObject->m_attachedTargets.size(); i++ )
	{
		RenderTarget::destroy( frameBufferObject->m_attachedTargets[ i ] );
	}

	glDeleteFramebuffers( 1, &frameBufferObject->m_id );
	delete frameBufferObject;

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

	if ( false == this->attachTargetTemp( renderTarget ) )
	{
		return false;
	}

	if ( RenderTarget::RT_DEPTH == renderTarget->getType() || RenderTarget::RT_SHADOW == renderTarget->getType() )
	{
		this->m_depthBuffer = renderTarget->getId();
		this->m_depthTarget = renderTarget;
	}
	else if ( RenderTarget::RT_COLOR == renderTarget->getType() )
	{
		GLuint id = renderTarget->getId();
		GLenum colorAttachment = GL_COLOR_ATTACHMENT0 + this->m_colorBufferTargets.size();

		this->m_colorBufferTargets.push_back( colorAttachment );
	}

	this->m_attachedTargets.push_back( renderTarget );

	return true;
}

bool
FrameBufferObject::attachTargetTemp( RenderTarget* renderTarget )
{
	if ( RenderTarget::RT_DEPTH == renderTarget->getType() || RenderTarget::RT_SHADOW == renderTarget->getType() )
	{
		// add this as a depth-attachment to get correct depth-visibility in our deferred rendering
		glFramebufferTexture( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, renderTarget->getId(), 0 );

#ifdef CHECK_GL_ERRORS
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::attachTargetTemp DEPTH: glFramebufferTexture failed ";
			return false;
		}
#endif
	}
	else if ( RenderTarget::RT_COLOR == renderTarget->getType() )
	{
		GLuint id = renderTarget->getId();
		GLenum colorAttachment = GL_COLOR_ATTACHMENT0 + this->m_colorBufferTargets.size();

		glFramebufferTexture2D( GL_FRAMEBUFFER, colorAttachment, GL_TEXTURE_2D, id, 0 );
#ifdef CHECK_GL_ERRORS
		if ( false == GLUtils::peekErrors() )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::attachTargetTemp COLOR: glFramebufferTexture failed";
			return false;
		}
#endif
	}

	return true;
}

bool
FrameBufferObject::bind()
{
	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, this->m_id );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::bind: glBindFramebuffer failed";
		return false;
	}
#endif

	return true;
}

bool
FrameBufferObject::unbind()
{
	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::unbind: glBindFramebuffer( 0 ) failed";
		return false;
	}
#endif

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
FrameBufferObject::drawAllBuffers()
{
	// activate multiple drawing to our color targets targets
	glDrawBuffers( this->m_colorBufferTargets.size(), &this->m_colorBufferTargets[ 0 ] );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::drawAllBuffers: glDrawBuffers failed";
		return false;
	}
#endif

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

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::drawAllBuffers: glDrawBuffers failed";
		return false;
	}
#endif

	return true;
}

bool
FrameBufferObject::drawBuffer( unsigned int index )
{
	// activate multiple drawing to our color targets targets
	glDrawBuffer( this->m_colorBufferTargets[ index ] );

#ifdef CHECK_GL_ERRORS
	if ( false == GLUtils::peekErrors() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "FrameBufferObject::drawBuffer: glDrawBuffer failed";
		return false;
	}
#endif

	return true;
}

bool
FrameBufferObject::drawNone()
{
	glDrawBuffer( GL_NONE );
	glReadBuffer( GL_NONE );

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
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	return true;
}

bool
FrameBufferObject::checkStatus()
{
#ifdef CHECK_GL_ERRORS
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
	
	return false;
#else
	return true;
#endif
}
