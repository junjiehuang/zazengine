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
#include "../Program/UniformBlock.h"

#include <iostream>
#include <algorithm>

#define MRT_COUNT 3

using namespace std;

DRRenderer::DRRenderer()
	: Renderer( )
{
	this->m_gBufferFbo = NULL;

	this->m_progGeomStage = NULL;
	this->m_vertGeomStage = NULL;
	this->m_fragGeomStage = NULL;

	this->m_progSkyBox = NULL;
	this->m_vertSkyBox = NULL;
	this->m_fragSkyBox = NULL;

	this->m_progLightingStage = NULL;
	this->m_vertLightingStage = NULL;
	this->m_fragLightingStage = NULL;

	this->m_progLightingNoShadowStage = NULL;
	this->m_fragLightingNoShadowStage = NULL;

	this->m_progShadowMapping = NULL;
	this->m_vertShadowMapping = NULL;
	this->m_fragShadowMapping = 0;

	this->m_shadowMappingFB = NULL;

	this->m_transformsBlock = NULL;
	this->m_cameraBlock = NULL;
	this->m_lightBlock = NULL;
	this->m_materialBlock = NULL;

	this->m_unitCubeMatrix = glm::mat4(
		0.5, 0.0, 0.0, 0.0, 
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
	);
}

DRRenderer::~DRRenderer()
{
}

bool
DRRenderer::initialize( const boost::filesystem::path& pipelinePath )
{
	cout << "Initializing Deferred Renderer..." << endl;

	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );					// Black Background
	glClearDepth( 1.0f );									// Depth Buffer Setup

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );	// Really Nice Perspective Calculations

	glEnable( GL_DEPTH_TEST );								// Enables Depth Testing
	glDepthFunc( GL_LESS );									// The Type Of Depth Testing To Do
	glDepthMask( GL_TRUE );

	// Cull triangles which normal is not towards the camera
	glEnable( GL_CULL_FACE );

	glEnable( GL_TEXTURE_2D );

	if ( false == this->initGBuffer() )
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

	if ( this->m_progSkyBox )
	{
		if ( this->m_vertSkyBox )
		{
			this->m_progSkyBox->detachShader( this->m_vertSkyBox );

			delete this->m_vertSkyBox;
			this->m_vertSkyBox = NULL;
		}

		if ( this->m_fragSkyBox )
		{
			this->m_progSkyBox->detachShader( this->m_fragSkyBox );

			delete this->m_fragSkyBox;
			this->m_fragSkyBox = NULL;
		}

		delete this->m_progSkyBox;
		this->m_progSkyBox = NULL;
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
	FrameBufferObject::destroy( this->m_gBufferFbo );
	
	// deleting shadow-map pool
	RenderTarget::cleanup();

	GeomSkyBox::shutdown();

	cout << "Shutting down Deferred Renderer finished" << endl;

	return true;
}

bool
DRRenderer::initGBuffer()
{
	this->m_gBufferFbo = FrameBufferObject::create();
	if ( NULL == this->m_gBufferFbo )
	{
		return false;
	}

	// needs to be bound to attach render-targets
	// the order of the following calls is important and depermines the index
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}
	
	// render-target at index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// render-target at index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// render-target at index 2: positions in viewing-coords (Eye-Space)
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR ) )		
	{
		return false;
	}

	// render-target at index 3: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH ) )		
	{
		return false;
	}

	// IMPORTANT: don't check status until all necessary buffers are created
	// and attached to the according fbo (e.g. after each attach) otherwise
	// it will be incomplete at some point and will fail.
	// check status of FBO AFTER all buffers are set-up and attached
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// unbind => switch back to default framebuffer
	if ( false == this->m_gBufferFbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::createMrtBuffer( RenderTarget::RenderTargetType targetType )
{
	RenderTarget* renderTarget = RenderTarget::create( ( GLsizei ) this->m_camera->getWidth(), 
		( GLsizei ) this->m_camera->getHeight(), targetType );
	if ( NULL == renderTarget )
	{
		return false;
	}

	if ( false == this->m_gBufferFbo->attachTarget( renderTarget ) )
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

	this->m_progSkyBox = Program::createProgram( "SkyBoxProgram" );
	if ( 0 == this->m_progSkyBox )
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

	this->m_vertSkyBox = Shader::createShader( Shader::VERTEX_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/skyBoxVert.glsl" );
	if ( 0 == this->m_vertSkyBox )
	{
		cout << "ERROR in DRRenderer::initGeomStage: coulnd't create fragment-shader - exit" << endl;
		return false;
	}

	this->m_fragSkyBox = Shader::createShader( Shader::FRAGMENT_SHADER, pipelinePath.generic_string() + "/dr/stages/geom/skyBoxFrag.glsl" );
	if ( 0 == this->m_fragSkyBox )
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

	if ( false == this->m_vertSkyBox->compile() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: vertex shader compilation failed - exit" << endl;
		return false;
	}

	if ( false == this->m_fragSkyBox->compile() )
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

	if ( false == this->m_progSkyBox->attachShader( this->m_vertSkyBox ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching vertex shader to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progSkyBox->attachShader( this->m_fragSkyBox ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: attaching fragment shader to program failed - exit" << endl;
		return false;
	}

	// IMPORANT: setting frag-data location is done bevore linking, otherwise linking fails
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
	if ( false == this->m_progGeomStage->bindFragDataLocation( 2, "out_position" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding fragment-data location failed - exit" << endl;
		return false;
	}

	// IMPORANT: setting frag-data location is done bevore linking, otherwise linking fails
	// sky-box only writes diffuse-color
	if ( false == this->m_progSkyBox->bindFragDataLocation( 0, "out_diffuse" ) )
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

	if ( false == this->m_progSkyBox->bindAttribLocation( 0, "in_vertPos" ) )
	{
		cout << "ERROR in DRRenderer::initGeomStage: binding attribute location to program failed - exit" << endl;
		return false;
	}

	if ( false == this->m_progGeomStage->link() )
	{
		cout << "ERROR in DRRenderer::initGeomStage: linking program failed - exit" << endl;
		return false;
	}
	
	if ( false == this->m_progSkyBox->link() )
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

	// shadow-mapping is done by rendering the depth of the current processing light into an own FBO
	this->m_shadowMappingFB = FrameBufferObject::create();
	if ( NULL == this->m_shadowMappingFB )
	{
		return false;
	}

	// bind the shadow FBO to init stuff
	if ( false == this->m_shadowMappingFB->bind() )
	{
		cout << "ERROR in DRRenderer::initShadowMapping: coulnd't bind shadow FBO - exit" << endl;
		return false;
	}

	// IMPORTANT: disable drawing&reading from this fbo is important, otherwise will fail as incomplete with depth-only attachment
	// no drawing & reading in shadow-fbo, set initial, fbo will keep this state, no need to set it every bind
	this->m_shadowMappingFB->drawNone();

	// back to default frame-buffer
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

	// IMPORANT: setting frag-data location is done bevore linking, otherwise linking fails
	if ( false == this->m_progShadowMapping->bindFragDataLocation( 0, "fragDepth" ) )
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
	if ( false == this->m_progSkyBox->bindUniformBlock( this->m_transformsBlock ) )
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

	// lighting-data & camera-data go just to lighting stage program
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

	// TODO clean-up: different approach in future
	GeomSkyBox::getRef().setTransformBlock( this->m_transformsBlock );

	return true;
}

bool
DRRenderer::renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	if ( false == this->doGeometryStage( instances, lights ) )
	{
		return false;
	}

	if ( false == this->doLightingStage( instances, lights ) )
	{
		return false;
	}

// if checking of gl-errors is deactivated check once per frame for errors
#ifndef CHECK_GL_ERROR
	GLenum status;
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "ERROR ... in DRRenderer::renderFrame: glGetError reported an error: " << gluErrorString( status ) << endl;
		return false;
	}
#endif

	this->frame++;

	return true;
}

bool
DRRenderer::doGeometryStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// render to g-buffer FBO
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_camera->restoreViewport();

	// clear all targets for the new frame
	if ( false == this->m_gBufferFbo->clearAll() )
	{
		return false;
	}

	// render sky-box first with disabled depth-write => will be behind all other objects
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

	// enable rendering to all render-targets in geometry-stage
	if ( false == this->m_gBufferFbo->drawAllBuffers() )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// draw all geometry from cameras viewpoint AND apply materials
	if ( false == this->renderInstances( this->m_camera, instances, this->m_progGeomStage, true ) )
	{
		return false;
	}

	// switch back to default framebuffer
	if ( false == this->m_gBufferFbo->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderSkyBox()
{
	// skip sky-box rendering if not present
	if ( false == GeomSkyBox::isPresent() )
	{
		return true;
	}

	// only render to buffer with index 0 which is diffuse-color only
	// IMPORANT: sky-box doesn't render normals, depth and other stuff
	//		because lighting and other effects won't be applied, only diffuse color matters
	if ( false == this->m_gBufferFbo->drawBuffer( 0 ) )
	{
		return false;
	}

	// check FBO-status
	if ( false == this->m_gBufferFbo->checkStatus() )
	{
		return false;
	}

	// sky-box rendering uses its own program
	if ( false == this->m_progSkyBox->use() )
	{
		cout << "ERROR in DRRenderer::renderSkyBox: using program failed - exit" << endl;
		return false;
	}

	this->m_progSkyBox->setUniformInt( "SkyBoxCubeMap", 0 );

	// render the geometry
	GeomSkyBox::getRef().render();

	return true;
}

bool
DRRenderer::doLightingStage( std::list<Instance*>& instances, std::list<Light*>& lights )
{
	// IMPORANT: when other fbos are used in this pipeline, they are always unbound after 
	//		usage so it is ensured that we are rendering to the default framebuffer now
	// clear default framebuffer
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glm::vec4 cameraRectangle;
	cameraRectangle[ 0 ] = ( float ) this->m_camera->getWidth();
	cameraRectangle[ 1 ] = ( float ) this->m_camera->getHeight();

	// bind camera uniform-block to update data
	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload world-orientation of camera ( its model-matrix )
	if ( false == this->m_cameraBlock->updateMat4( this->m_camera->getModelMatrix(), 0 ) )
	{
		return false;
	}
	// upload view-matrix of camera (need to transform e.g. light-world position in EyeCoords/Viewspace)
	if ( false == this->m_cameraBlock->updateMat4( this->m_camera->getViewMatrix(), 64 ) )
	{
		return false;
	}
	// upload camera-rectangle
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
	// different programs could be active according whether light is shadow-caster or not
	// IMPORANT: unbinding textures will fail when shadowmap is not used within program due to no shadow-mapping
	Program* activeLightingProgram = NULL;

	// light is shadow-caster: render shadow map for this light
	if ( light->isShadowCaster() )
	{
		if ( false == this->renderShadowMap( instances, light ) )
		{
			return false;
		}

		activeLightingProgram = this->m_progLightingStage;
	}
	// light is no shadow-caster => choose different lighting-stage program
	else
	{
		activeLightingProgram = this->m_progLightingNoShadowStage;
	}

	// activate lighting-stage shader
	if ( false == activeLightingProgram->use() )
	{
		cout << "ERROR in DRRenderer::renderLight: using program failed - exit" << endl;
		return false;
	}

	// lighting stage program need all buffers of g-buffer bound as textures
	if ( false == this->m_gBufferFbo->bindAllTargets() )
	{
		return false;
	}

	// tell lighting program that diffusemap is bound to texture-unit 0
	activeLightingProgram->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	activeLightingProgram->setUniformInt( "NormalMap", 1 );
	// tell lighting program that generic map is bound to texture-unit 2
	activeLightingProgram->setUniformInt( "PositionMap", 2 );
	// tell lighting program that depth-map of scene is bound to texture-unit MRT_COUNT
	activeLightingProgram->setUniformInt( "DepthMap", MRT_COUNT );
	
	glm::vec4 lightConfig;
	glm::mat4 lightSpaceUnit;

	lightConfig[ 0 ] = ( float ) light->getType();
	lightConfig[ 1 ] = light->getFalloff();
	lightConfig[ 2 ] = light->isShadowCaster();
		
	// bind light uniform-block to update data of this light
	if ( false == this->m_lightBlock->bindBuffer() )
	{
		return false;
	}

	// upload light-config
	if ( false == this->m_lightBlock->updateVec4( lightConfig, 0 ) )
	{
		return false;
	}
	// upload light-model matrix = orientation of the light in the world
	if ( false == this->m_lightBlock->updateMat4( light->getModelMatrix(), 16 ) )
	{
		return false;
	}

	// bind shadow-map when light is shadow-caster
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
		lightSpaceUnit = this->m_unitCubeMatrix * light->getVPMatrix();

		if ( false == this->m_lightBlock->updateMat4( lightSpaceUnit, 80 ) )
		{
			return false;
		}
	}

	// QUESTION: due to a but only bind was called instead of bindBuffer but it worked!! why?
	// update projection-matrix because need ortho-projection for full-screen quad
	this->m_transformsBlock->bindBuffer();
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_camera->createOrthoProj( true, true );
	if ( false == this->m_transformsBlock->updateMat4( orthoMat, 64 ) )
	{
		return false;
	}

	// TODO: need to set opengl to additiveley blend light-passes
	// alpha? color modulate?...

	// render light boundary (for now only full screen-quad)
	light->getBoundingGeometry()->render();

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<Instance*>& instances, Light* light )
{
	// use shadow-mapping fbo to render depth of light view-point to
	if ( false == this->m_shadowMappingFB->bind() )
	{
		return false;
	}

	// attach this shadow-map temporarily to the fbo (other light will use the same fbo)
	if ( false == this->m_shadowMappingFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	// check status now
	// IMPORANT: don't check too early
	if ( false == this->m_shadowMappingFB->checkStatus() )
	{
		return false;
	}

	// clear bound buffer
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// use shadow-mapping program
	if ( false == this->m_progShadowMapping->use() )
	{
		cout << "ERROR in DRRenderer::renderShadowMap: using program failed - exit" << endl;
		return false;
	}

	// IMPORTANT: need to set the viewport for each shadow-map, because resolution can be different for each
	light->restoreViewport();

	// render scene from view of camera - don't apply material, we need only depth
	if ( false == this->renderInstances( light, instances, this->m_progShadowMapping, false ) )
	{
		return false;
	}

	// back to default framebuffer
	if ( false == this->m_shadowMappingFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderInstances( Viewer* viewer, list<Instance*>& instances, Program* currentProgramm, bool applyMaterial )
{
	// bind transform uniform-block to update model-, view & projection transforms
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	// update projection because each viewer can have different projection-transform
	if ( false == this->m_transformsBlock->updateMat4( viewer->getProjMatrix(), 64 ) )
	{
		return false;
	}

	list<Instance*>::iterator iter = instances.begin();
	while ( iter != instances.end() )
	{
		Instance* instance = *iter++;

		// activate material only when there is one present & render-pass enforces usage of materials
		if ( instance->material && applyMaterial )
		{
			// need to bind material uniform-block to update data
			if ( false == this->m_materialBlock->bindBuffer() )
			{
				return false;
			}

			// activate material
			if ( false == instance->material->activate( this->m_materialBlock, currentProgramm ) )
			{
				return false;
			}

			// back to transform uniform-block because is needed in the actual rendering of geometry (modelview calculation)
			if ( false == this->m_transformsBlock->bindBuffer() )
			{
				return false;
			}
		}

		// render geometry of this instance
		if ( false == this->renderGeom( viewer, instance->geom, instance->getModelMatrix() ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderGeom( Viewer* viewer, GeomType* geom, const glm::mat4& rootModelMatrix )
{
	// IMPORANT: OpenGL applies last matrix in multiplication as first transformation to object
	// apply model-transformations recursive
	glm::mat4 modelMatrix = rootModelMatrix * geom->getModelMatrix();
	const std::vector<GeomType*>& children = geom->getChildren();

	if ( children.size() )
	{
		for ( unsigned int i = 0; i < children.size(); i++ )
		{
			GeomType* child = children[ i ];
			// recursively process children
			if ( false == this->renderGeom( viewer, children[ i ], modelMatrix ) )
			{
				return false;
			}
		}
	}
	else
	{
		// cull objects using the viewers point of view
		Viewer::CullResult cullResult = viewer->cullBB( geom->getBBMin(), geom->getBBMax() );
		if ( Viewer::OUTSIDE != cullResult )
		{
			// calculate modelView-Matrix
			glm::mat4 modelViewMatrix = viewer->getViewMatrix() * modelMatrix;
			// IMPORTANT: normal-vectors are transformed different than vertices
			// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
			// in other words: only the rotations are applied to normals and they are guaranteed to leave
			// normalized normals at unit length. THIS METHOD ONLY WORKS WHEN NO NON UNIFORM SCALING IS APPLIED
			glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( modelViewMatrix ) );

			// update model-view matrix
			if ( false == this->m_transformsBlock->updateMat4( modelViewMatrix, 0 ) )
			{
				return false;
			}
			// update model-view matrix for normals
			if ( false == this->m_transformsBlock->updateMat4( normalModelViewMatrix, 128 ) )
			{
				return false;
			}

			// render geometry
			return geom->render();
		}
	}

	return true;
}
