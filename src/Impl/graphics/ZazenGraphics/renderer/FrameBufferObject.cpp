#include "FrameBufferObject.h"

#include <iostream>

#include <assert.h>

using namespace std;

#define CHECK_FRAMEBUFFER_STATUS( status ) \
{\
 status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT); \
 switch(status) { \
 case GL_FRAMEBUFFER_COMPLETE: \
   break; \
 case GL_FRAMEBUFFER_UNSUPPORTED: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_UNSUPPORTED\n");\
    /* you gotta choose different formats */ \
   assert(0); \
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT\n");\
   break; \
  case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n");\
   break; \
 case GL_FRAMEBUFFER_BINDING: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_BINDING\n");\
   break; \
 default: \
   /* programming error; will fail on all hardware */ \
   assert(0); \
 }\
}

FrameBufferObject*
FrameBufferObject::create()
{
	GLuint id = 0;
	GLenum status;

	// generate the id of our frame-buffer-object
	glGenFramebuffers( 1, &id );
	if ( GL_NO_ERROR != ( status = glGetError() )  )
	{
		cout << "ERROR in FrameBufferObject::create: glGenFramebuffers failed with " << gluErrorString( status ) << " - exit" << endl;
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
	GLenum status;

	// ignore when already added
	for ( unsigned int i = 0; i < this->m_attachedTargets.size(); i++ )
	{
		if ( renderTarget->getId() == this->m_attachedTargets[ i ]->getId() )
		{
			return true;
		}
	}

	if ( RenderTarget::RT_DEPTH == renderTarget->getType() || RenderTarget::RT_SHADOW == renderTarget->getType() )
	{
		// add this as a depth-attachment to get correct depth-visibility in our deferred rendering
		glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderTarget->getId(), 0 );
		CHECK_FRAMEBUFFER_STATUS( status );
		if ( GL_FRAMEBUFFER_COMPLETE != status )
		{
			cout << "ERROR in FrameBufferObject::attachTarget: framebuffer error for depth-buffer: " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}

		this->m_depthBuffer = renderTarget->getId();
		this->m_depthTarget = renderTarget;
	}
	else if ( RenderTarget::RT_COLOR == renderTarget->getType() )
	{
		GLuint id = renderTarget->getId();
		GLenum colorAttachment = GL_COLOR_ATTACHMENT0 + this->m_colorBufferTargets.size();

		glFramebufferTexture2D( GL_FRAMEBUFFER, colorAttachment, GL_TEXTURE_2D, id, 0 );
		CHECK_FRAMEBUFFER_STATUS( status );
		if ( GL_FRAMEBUFFER_COMPLETE != status )
		{
			cout << "ERROR in DRRenderer::initMrtBuffer: framebuffer error: " << gluErrorString( status ) << " - exit" << endl;
			return false;
		}

		this->m_colorBufferTargets.push_back( colorAttachment );
		this->m_colorBuffers.push_back( id );
	}

	this->m_attachedTargets.push_back( renderTarget );

	return true;
}

bool
FrameBufferObject::bind()
{
	GLenum status;

	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, this->m_id );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in FrameBufferObject::bind: glBindFramebuffer failed with " << gluErrorString( status ) << endl;
		return false;
	}

	return true;
}

bool
FrameBufferObject::unbind()
{
	GLenum status;
	// bind the framebuffer of the geometry-stage
	glBindFramebuffer( GL_FRAMEBUFFER, 0 );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in FrameBufferObject::unbind: glBindFramebuffer failed with " << gluErrorString( status ) << endl;
		return false;
	}

	return true;
}

bool
FrameBufferObject::bindAllTargets()
{
	// bind the mrts
	for ( unsigned int i = 0; i < this->m_attachedTargets.size(); i++ )
	{
		this->m_attachedTargets[ i ]->bind( i );
	}

	return true;
}

bool
FrameBufferObject::unbindAllTargets()
{
	// bind the mrts
	for ( unsigned int i = 0; i < this->m_attachedTargets.size(); i++ )
	{
		this->m_attachedTargets[ i ]->unbind();
	}

	return true;
}

bool
FrameBufferObject::drawAllBuffers()
{
	GLenum status;

	// activate multiple drawing to our color targets targets
	glDrawBuffers( this->m_colorBufferTargets.size(), &this->m_colorBufferTargets[ 0 ] );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in FrameBufferObject::drawAllBuffers: glDrawBuffers failed with " << gluErrorString( status ) << endl;
		return false;
	}

	// check framebuffer status, maybe something failed with glDrawBuffers
	CHECK_FRAMEBUFFER_STATUS( status );
	if ( GL_FRAMEBUFFER_COMPLETE != status )
	{
		cout << "ERROR in FrameBufferObject::drawAllBuffers: framebuffer error: " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

	return true;
}

bool
FrameBufferObject::drawBuffer( unsigned int index )
{
	GLenum status;

	// activate multiple drawing to our color targets targets
	glDrawBuffer( this->m_colorBufferTargets[ index ] );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in FrameBufferObject::drawBuffer: glDrawBuffer failed with " << gluErrorString( status ) << endl;
		return false;
	}

	// check framebuffer status, maybe something failed with glDrawBuffers
	CHECK_FRAMEBUFFER_STATUS( status );
	if ( GL_FRAMEBUFFER_COMPLETE != status )
	{
		cout << "ERROR in FrameBufferObject::drawBuffer: framebuffer error: " << gluErrorString( status ) << " - exit" << endl;
		return false;
	}

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
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	// set clear-color
	glClearColor( 0.0, 0.0, 0.0, 1.0 );
	// clear the colorbuffers AND our depth-buffer ( m_geometryDepth );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	return true;
}
