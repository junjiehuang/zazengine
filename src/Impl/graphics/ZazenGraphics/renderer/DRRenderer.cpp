/*
 * DRRenderer.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#include <GL/glew.h>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "DRRenderer.h"

#include "../Geometry/GeomSkyBox.h"
#include "../Material/UniformBlock.h"

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
	: Renderer( )
{
	this->m_fbo = 0;

	this->m_progGeomStage = 0;
	this->m_vertGeomStage = 0;
	this->m_fragGeomStage = 0;

	this->m_progLightingStage = 0;
	this->m_vertLightingStage = 0;
	this->m_fragLightingStage = 0;

	this->m_progLightingNoShadowStage = 0;
	this->m_fragLightingNoShadowStage = 0;

	this->m_progShadowMapping = 0;
	this->m_vertShadowMapping = 0;
	this->m_fragShadowMapping = 0;

	this->m_shadowMappingFB = 0;

	this->m_transformsBlock = 0;
	this->m_cameraBlock = 0;
	this->m_lightBlock = 0;
	this->m_materialBlock = 0;

	this->m_unitCubeMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0, 
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
	);

	this->m_displayMRT = false;
}

DRRenderer::~DRRenderer()
{
}

bool
DRRenderer::toggleDisplay()
{
	this->m_displayMRT = !this->m_displayMRT;

	return true;
}

bool
DRRenderer::initialize( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Renderer..." << endl;

	glShadeModel( GL_SMOOTH );								// Enable Smooth Shading
	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );					// Black Background
	glClearDepth( 1.0f );									// Depth Buffer Setup

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );	// Really Nice Perspective Calculations

	glEnable( GL_DEPTH_TEST );								// Enables Depth Testing
	glDepthFunc( GL_LESS );								// The Type Of Depth Testing To Do
	glDepthMask( GL_TRUE );

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	glEnable( GL_TEXTURE_2D );

	if ( false == this->initFBO() )
	{
		return false;
	}

	if ( false == this->initGeomStage( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initLightingStage( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initShadowMapping( pipelinePath ) )
	{
		return false;
	}

	if ( false == this->initUniformBlocks() )
	{
		return false;
	}

	cout << "Initializing Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::shutdown()
{
	cout << "Shutting down Deferred Renderer..." << endl;

	Program::unuse();

	// cleaning up shadow mapping
	if ( this->m_progShadowMapping )
	{
		if ( this->m_vertShadowMapping )
		{
			this->m_progShadowMapping->detachShader( this->m_vertShadowMapping );

			delete this->m_vertShadowMapping;
			this->m_vertShadowMapping = NULL;
		}

		if ( this->m_fragShadowMapping )
		{
			this->m_progShadowMapping->detachShader( this->m_fragShadowMapping );

			delete this->m_fragShadowMapping;
			this->m_fragShadowMapping = NULL;
		}

		delete this->m_progShadowMapping;
		this->m_progShadowMapping = NULL;
	}

	// cleaning up lighting stage
	if ( this->m_progLightingStage )
	{
		if ( this->m_vertLightingStage )
		{
			this->m_progLightingStage->detachShader( this->m_vertLightingStage );

			delete this->m_vertLightingStage;
			this->m_vertLightingStage = NULL;
		}

		if ( this->m_fragLightingStage )
		{
			this->m_progLightingStage->detachShader( this->m_fragLightingStage );

			delete this->m_fragLightingStage;
			this->m_fragLightingStage = NULL;
		}

		delete this->m_progLightingStage;
		this->m_progLightingStage = NULL;
	}
	
	// cleaning up lighting stage
	if ( this->m_progLightingNoShadowStage )
	{
		if ( this->m_fragLightingNoShadowStage )
		{
			this->m_progLightingNoShadowStage->detachShader( this->m_fragLightingNoShadowStage );

			delete this->m_fragLightingNoShadowStage;
			this->m_fragLightingNoShadowStage = NULL;
		}

		delete this->m_progLightingNoShadowStage;
		this->m_progLightingNoShadowStage = NULL;
	}

	// cleaning up geometry-stage
	if ( this->m_progGeomStage )
	{
		if ( this->m_vertGeomStage )
		{
			this->m_progGeomStage->detachShader( this->m_vertGeomStage );

			delete this->m_vertGeomStage;
			this->m_vertGeomStage = NULL;
		}

		if ( this->m_fragGeomStage )
		{
			this->m_progGeomStage->detachShader( this->m_fragGeomStage );

			delete this->m_fragGeomStage;
			this->m_fragGeomStage = NULL;
		}

		delete this->m_progGeomStage;
		this->m_progGeomStage = NULL;
	}

	// cleaning up uniform blocks
	if ( this->m_transformsBlock )
	{
		delete this->m_transformsBlock;
		this->m_transformsBlock = NULL;
	}

	if ( this->m_cameraBlock )
	{
		delete this->m_cameraBlock;
		this->m_cameraBlock = NULL;
	}

	if ( this->m_lightBlock )
	{
		delete this->m_lightBlock;
		this->m_lightBlock = NULL;
	}

	if ( this->m_materialBlock )
	{
		delete this->m_materialBlock;
		this->m_materialBlock = NULL;
	}

	FrameBufferObject::destroy( this->m_shadowMappingFB );

	// cleaning up framebuffer
	FrameBufferObject::destroy( this->m_fbo );
	
	GeomSkyBox::shutdown();

	cout << "Shutting down Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::initFBO()
{
	this->m_fbo = FrameBufferObject::create();
	if ( NULL == this->m_fbo )
	{
		return false;
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}
	
	// index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 2: position data
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 3: generic attributes
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// index 4: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH ) )		
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initGeomStage( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Geometry-Stage..." << endl;

	this->m_progGeomStage = Program::createProgram( "GeometryStageProgramm" );
	if ( 0 == this->m_progGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertGeomStage = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/geomVert.glsl" );
	if ( 0 == this->m_vertGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create vertex-shader - exit" << endl;
		return false;
	}

	this->m_fragGeomStage = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/geomFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertGeomStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragGeomStage->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->attachShader( this->m_vertGeomStage ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->attachShader( this->m_fragGeomStage ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	// setting frag-data location is done bevore linking
	if ( false == this->m_progGeomStage->bindFragDataLocation( 0, "out_diffuse" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 1, "out_normal" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 2, "out_generic1" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindFragDataLocation( 3, "out_generic2" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindAttribLocation( 1, "in_vertNorm" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindAttribLocation( 2, "in_texCoord" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->link() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: linking program failed - exit" << endl;
		return false;
	}

	cout << "Initializing Deferred Rendering Geometry-Stage finished" << endl;

	return true;
}

bool
DRRenderer::initLightingStage( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Lighting-Stage..." << endl;

	this->m_progLightingStage = Program::createProgram( "LightingStageProgramm" );
	if ( 0 == this->m_progLightingStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_progLightingNoShadowStage = Program::createProgram( "LightingNoShadowStageProgramm" );
	if ( 0 == this->m_progLightingNoShadowStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertLightingStage = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/lighting/lightVert.glsl" );
	if ( 0 == this->m_vertLightingStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create vertex-shader - exit" << endl;
		return false;
	}

	this->m_fragLightingStage = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/lighting/lightFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	this->m_fragLightingNoShadowStage = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/lighting/lightNoShadowFrag.glsl" );
	if ( 0 == this->m_fragGeomStage )
	{
		cout << "ERROR in DRRenderer::initLightingStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertLightingStage->compile() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragLightingStage->compile() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragLightingNoShadowStage->compile() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->attachShader( this->m_vertLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->attachShader( this->m_fragLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingStage->link() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: linking program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingNoShadowStage->attachShader( this->m_vertLightingStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingNoShadowStage->attachShader( this->m_fragLightingNoShadowStage ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingNoShadowStage->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initLightingStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progLightingNoShadowStage->link() )
	{
		cout << "ERROR in DRRenderer::initLightingStage: linking program failed - exit" << endl;
		return false;
	}
	cout << "Initializing Deferred Rendering Lighting-Stage finished" << endl;

	return true;
}

bool
DRRenderer::initShadowMapping( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Rendering Shadow-Mapping..." << endl;

	this->m_shadowMappingFB = FrameBufferObject::create();
	if ( NULL == this->m_shadowMappingFB )
	{
		return false;
	}

	if ( false == this->m_shadowMappingFB->bind() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't bind shadow FBO - exit" << endl;
		return false;
	}

	// no drawing & reading in shadow-fbo, set initial, fbo will keep this state, no need to set it every bind
	this->m_shadowMappingFB->drawNone();

	if ( false == this->m_shadowMappingFB->unbind() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't unbind shadow FBO - exit" << endl;
		return false;
	}

	this->m_progShadowMapping = Program::createProgram( "ShadowingStageProgramm" );
	if ( 0 == this->m_progShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create program - exit" << endl;
		return false;
	}

	this->m_vertShadowMapping = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/shadowing/shadowVert.glsl" );
	if ( 0 == this->m_vertShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create vertex shader - exit" << endl;
		return false;
	}

	this->m_fragShadowMapping = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/shadowing/shadowFrag.glsl" );
	if ( 0 == this->m_fragShadowMapping )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't create fragment shader - exit" << endl;
		return false;
	}

	if ( false == this->m_vertShadowMapping->compile() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragShadowMapping->compile() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: fragment shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->attachShader( this->m_vertShadowMapping ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->attachShader( this->m_fragShadowMapping ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	// setting frag-data location is done bevore linking
	if ( false == this->m_progShadowMapping->bindFragDataLocation( 0, "fragmentdepth" ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: binding fragment-data location failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progShadowMapping->link() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: linking program failed - exit" << endl;
		return false;
	}

	cout << "Initializing Deferred Rendering Shadow-Mapping finished" << endl;

	return true;
}

bool
DRRenderer::initUniformBlocks()
{
	this->m_transformsBlock = UniformBlock::createBlock( "transforms" );
	if ( 0 == this->m_transformsBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_cameraBlock = UniformBlock::createBlock( "camera" );
	if ( 0 == this->m_cameraBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_lightBlock = UniformBlock::createBlock( "light" );
	if ( 0 == this->m_lightBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	this->m_materialBlock = UniformBlock::createBlock( "material" );
	if ( 0 == this->m_materialBlock )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: creating uniform block failed - exit" << endl;
		return false;
	}

	// bind transformation-data to all programs
	if ( false == this->m_progShadowMapping->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progGeomStage->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingNoShadowStage->bindUniformBlock( this->m_transformsBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	// lighting-data & camera-data just to lighting stage program
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_lightBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingStage->bindUniformBlock( this->m_cameraBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingNoShadowStage->bindUniformBlock( this->m_lightBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}
	if ( false == this->m_progLightingNoShadowStage->bindUniformBlock( this->m_cameraBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	// material-data just go to geometry-stage program
	if ( false == this->m_progGeomStage->bindUniformBlock( this->m_materialBlock ) )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: failed binding uniform block - exit" << endl;
		return false;
	}

	/* IMPORTANT: found this in forums: 
		On ATI hardware, you have to call

		glGetUniformBlockIndex( program, blockName )
		glUniformBlockBinding( program, blockIndex, slot )

		BEFORE

		glBindBufferBase( GL_UNIFORM_BUFFER, slot, UBO )

		On NVIDIA hardware, the other way around is fine.

		THIS IMPLIES: call bind() method on UniformBlock AFTER calling bindUniformBlock() on programm
	*/
	// bind uniform blocks
	if ( false == this->m_transformsBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_cameraBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_lightBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}
	if ( false == this->m_materialBlock->bind() )
	{
		cout << "ERROR in DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" << endl;
		return false;
	}

	return true;
}

bool
DRRenderer::createMrtBuffer( RenderTarget::RenderTargetType targetType )
{
	RenderTarget* renderTarget = RenderTarget::create( ( GLsizei ) this->m_camera->getWidth(), ( GLsizei ) this->m_camera->getHeight(), targetType );
	if ( NULL == renderTarget )
	{
		return false;
	}

	if ( false == this->m_fbo->attachTarget( renderTarget ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	
	if ( false == this->renderGeometryStage( instances, lights ) )
	{
		return false;
	}

	if ( false == this->m_displayMRT ) 
	{
		if ( false == this->renderLightingStage( instances, lights ) )
		{
			return false;
		}
	}
	else
	{
		this->m_camera->restoreViewport();

		glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
		glClearColor( 0.0, 0.0, 0.0, 1.0 );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 0 ]->getId(), 0 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 1 ]->getId(), 1 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedTargets()[ 2 ]->getId(), 2 ) )
		{
			return false;
		}
		if ( false == this->showTexture( this->m_fbo->getAttachedDepthTarget()->getId(), 3 ) )
		{
			return false;
		}
	}

	this->frame++;

	return true;
}

// TODO cleanup!
bool
DRRenderer::renderGeometryStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	this->m_camera->restoreViewport();

	if ( false == this->m_fbo->clearAll() )
	{
		return false;
	}

	// TODO check if we really need this
	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	if ( false == this->renderSkyBox() )
	{
		return false;
	}

	// activate geometry-stage program
	if ( false == this->m_progGeomStage->use() )
	{
		cout << "ERROR in DRRenderer::renderGeometryStage: using shadow mapping program failed - exit" << endl;
		return false;
	}

	// TODO check if we really need this
	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	if ( false == this->m_fbo->drawAllBuffers() )
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	// draw all geometry - apply materials but don't render transparency
	if ( false == this->renderInstances( this->m_camera, instances, this->m_progGeomStage, true, false ) )
	{
		return false;
	}

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderSkyBox()
{
	if ( false == GeomSkyBox::isPresent() )
	{
		return true;
	}

	if ( false == Program::unuse() )
	{
		cout << "ERROR in DRRenderer::renderSkyBox: failed deactivating program " << endl;
		return false;
	}

	if ( false == this->m_fbo->bind() )
	{
		return false;
	}

	if ( false == this->m_fbo->drawBuffer( 0 ) )
	{
		return false;
	}

	if ( false == this->m_fbo->checkStatus() )
	{
		return false;
	}

	GeomSkyBox::getRef().render();

	if ( false == this->m_fbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderLightingStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// clear main framebuffer
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glm::vec4 cameraRectangle;
	cameraRectangle[ 0 ] = ( float ) this->m_camera->getWidth();
	cameraRectangle[ 1 ] = ( float ) this->m_camera->getHeight();

	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload world-orientation of camera ( its model-matrix )
	if ( false == this->m_cameraBlock->updateMat4( this->m_camera->getMatrix(), 0 ) )
	{
		return false;
	}
	// upload view-matrix of camera (need to transform e.g. light-world position in EyeCoords/Viewspace)
	if ( false == this->m_cameraBlock->updateMat4( this->m_camera->m_viewMatrix, 64 ) )
	{
		return false;
	}
	// upload camera-rectangel
	if ( false == this->m_cameraBlock->updateVec4( cameraRectangle, 128 ) )
	{
		return false;
	}

	// render the contribution of each light to the scene
	std::list<Light*>::iterator iter = lights.begin();
	while ( iter != lights.end() )
	{
		Light* light = *iter++;

		if ( false == this->renderLight( instances, light ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderLight( std::list<Instance*>& instances, Light* light )
{
	Program* activeLightingProgram = NULL;

	if ( light->isShadowCaster() )
	{
		if ( false == this->renderShadowMap( instances, light ) )
		{
			return false;
		}

		activeLightingProgram = this->m_progLightingStage;
	}
	else
	{
		activeLightingProgram = this->m_progLightingNoShadowStage;
	}
	
	// activate lighting-stage (with shadowing) shader
	if ( false == activeLightingProgram->use() )
	{
		cout << "ERROR in DRRenderer::renderLightingStage: using program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fbo->bindAllTargets() )
	{
		return false;
	}

	// TODO remove: can use layout in shader to bind
	// tell lighting program that diffusemap is bound to texture-unit 0
	activeLightingProgram->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	activeLightingProgram->setUniformInt( "NormalMap", 1 );
	// tell lighting program that generic map is bound to texture-unit 2
	activeLightingProgram->setUniformInt( "GenericMap1", 2 );
	// tell lighting program that generic map is bound to texture-unit 3
	activeLightingProgram->setUniformInt( "GenericMap2", 3 );
	// tell lighting program that depth-map of scene is bound to texture-unit MRT_COUNT
	activeLightingProgram->setUniformInt( "DepthMap", MRT_COUNT );
	
	glm::vec4 lightConfig;
	glm::mat4 lightSpaceUnit;

	lightConfig[ 0 ] = ( float ) light->getType();
	lightConfig[ 1 ] = light->getFalloff();
	lightConfig[ 2 ] = light->isShadowCaster();
		
	if ( false == this->m_lightBlock->bindBuffer() )
	{
		return false;
	}

	if ( false == this->m_lightBlock->updateVec4( lightConfig, 0 ) )
	{
		return false;
	}
	if ( false == this->m_lightBlock->updateVec4( light->getColor(), 16 ) )
	{
		return false;
	}
	if ( false == this->m_lightBlock->updateMat4( light->m_modelMatrix, 32 ) )
	{
		return false;
	}

	if ( light->isShadowCaster() )
	{
		// tell program that the shadowmap of spot/directional-light will be available at texture unit MRT_COUNT + 1
		activeLightingProgram->setUniformInt( "ShadowMap", MRT_COUNT + 1 );

		if ( false == light->getShadowMap()->bind( MRT_COUNT + 1 ) )
		{
			return false;
		}

		// calculate the light-space projection matrix
		// multiplication with unit-cube is first because has to be carried out the last
		lightSpaceUnit = this->m_unitCubeMatrix * light->m_VPMatrix;

		if ( false == this->m_lightBlock->updateMat4( lightSpaceUnit, 96 ) )
		{
			return false;
		}
	}

	this->m_transformsBlock->bind();
	// update projection-matrix because changed to orthogonal-projection
	glm::mat4 orthoMat = this->m_camera->createOrthoProj( false, true );
	if ( false == this->m_transformsBlock->updateMat4( orthoMat, 64 ) )
	{
		return false;
	}

	// TODO: need to set opengl to additiveley blend light-passes
	// alpha? color modulate?...

	// render quad
	glBegin( GL_QUADS );
		glVertex2f( 0, 0 );
		glVertex2f( 0, ( float ) this->m_camera->getHeight() );
		glVertex2f( ( float ) this->m_camera->getWidth(), ( float ) this->m_camera->getHeight() );
		glVertex2f( ( float ) this->m_camera->getWidth(), 0 );
	glEnd();

	if ( light->isShadowCaster() )
	{
		if ( false == light->getShadowMap()->unbind() )
		{
			return false;
		}
	}

	if ( false == this->m_fbo->unbindAllTargets() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<Instance*>& instances, Light* light )
{
	// Rendering offscreen
	if ( false == this->m_shadowMappingFB->bind() )
	{
		return false;
	}

	// TODO won't work with multiple shadow-maps! do different when multiple lightsources implemented
	if ( false == this->m_shadowMappingFB->attachTarget( light->getShadowMap() ) )
	{
		return false;
	}

	// no all set up for checking completeness
	if ( false == this->m_shadowMappingFB->checkStatus() )
	{
		return false;
	}

	// clear bound buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	if ( false == this->m_progShadowMapping->use() )
	{
		cout << "ERROR in DRRenderer::renderShadowMap: using program failed - exit" << endl;
		return false;
	}

	// IMPORTANT: need to set the viewport for each shadow-map, because resolution can be different for each
	light->restoreViewport();

	// render scene from view of camera - don't apply material, don't render transparency
	if ( false == this->renderInstances( light, instances, this->m_progShadowMapping, false, false ) )
	{
		return false;
	}

	// back to window-system framebuffer
	if ( false == this->m_shadowMappingFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderInstances( Viewer* viewer, list<Instance*>& instances, Program* currentProgramm, 
		bool applyMaterial, bool transparencyPass )
{
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	if ( false == this->m_transformsBlock->updateMat4( viewer->m_projectionMatrix, 64 ) )
	{
		return false;
	}

	list<Instance*>::iterator iter = instances.begin();
	while ( iter != instances.end() )
	{
		Instance* instance = *iter++;
		Material* activeMaterial = NULL;

		if ( instance->material && applyMaterial )
		{
			if ( transparencyPass )
			{
				if ( instance->material->getType() != Material::MATERIAL_TRANSPARENT )
				{
					continue;
				}
			}
			else
			{
				if ( instance->material->getType() == Material::MATERIAL_TRANSPARENT )
				{
					continue;
				}
			}

			activeMaterial = instance->material;
		}

		if ( activeMaterial )
		{
			if ( false == this->m_materialBlock->bindBuffer() )
			{
				return false;
			}

			if ( false == instance->material->activate( this->m_materialBlock, currentProgramm ) )
			{
				return false;
			}

			if ( false == this->m_transformsBlock->bindBuffer() )
			{
				return false;
			}
		}

		if ( false == this->renderGeom( viewer, instance->geom, instance->getMatrix() ) )
		{
			return false;
		}

		if ( activeMaterial )
		{
			instance->material->deactivate();
		}
	}

	return true;
}

// NOTE OpenGL applies last matrix in multiplication as first transformation to object
bool
DRRenderer::renderGeom( Viewer* viewer, GeomType* geom, const glm::mat4& rootModelMatrix )
{
	glm::mat4 modelMatrix = rootModelMatrix * geom->m_modelMatrix;

	if ( geom->children.size() )
	{
		for ( unsigned int i = 0; i < geom->children.size(); i++ )
		{
			GeomType* child = geom->children[ i ];
			if ( false == this->renderGeom( viewer, geom->children[ i ], modelMatrix ) )
			{
				return false;
			}
		}
	}
	else
	{
		Viewer::CullResult cullResult = viewer->cullBB( geom->getBBMin(), geom->getBBMax() );
		if ( Viewer::OUTSIDE != cullResult )
		{
			// calculate modelView-Matrix
			glm::mat4 modelViewMatrix = viewer->m_viewMatrix * modelMatrix;
			// normal-vectors are transformed different than vertices
			// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
			// in other words: only the rotations are applied to normals and they are guaranteed to leave
			// normalized normals at unit length. THIS METHOD ONLY WORKS WHEN NO NON UNIFORM SCALING IS APPLIED
			glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( modelViewMatrix ) );

			if ( false == this->m_transformsBlock->updateMat4( modelViewMatrix, 0 ) )
			{
				return false;
			}
			if ( false == this->m_transformsBlock->updateMat4( normalModelViewMatrix, 128 ) )
			{
				return false;
			}

			return geom->render();
		}
	}

	return true;
}

bool
DRRenderer::showTexture( GLuint texID, int quarter )
{
	GLint status;

	if ( false == Program::unuse() )
	{
		cout << "ERROR in DRRenderer::showTexture: failed deactivating program " << endl;
		return false;
	}

	glm::mat4 orthoMat = this->m_camera->createOrthoProj( false, true );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( orthoMat ) );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glActiveTexture( GL_TEXTURE0 );
	glBindTexture( GL_TEXTURE_2D, texID );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR in DRRenderer::showTexture: glBindTexture failed with " << gluErrorString( status ) << endl;
		return false;
	}

	int counter = 0;
	float height = ( float ) this->m_camera->getHeight() / 2.0f;
	float width = ( float ) this->m_camera->getWidth() / 2.0f;

	for ( int i = 0; i < 2; i++ )
	{
		for ( int j = 0; j < 2; j++ )
		{
			if ( counter == quarter )
			{
				// render quad
				glBegin( GL_QUADS );
					glTexCoord2f( 0.0f, 1.0f ); glVertex2f( width * j, height * i );
					glTexCoord2f( 0.0f, 0.0f ); glVertex2f( width * j, height * i + height );
					glTexCoord2f( 1.0f, 0.0f ); glVertex2f( width * j + width, height * i + height );
					glTexCoord2f( 1.0f, 1.0f ); glVertex2f( width * j + width, height * i );
				glEnd();
			}

			counter++;
		}
	}

	return true;
}
