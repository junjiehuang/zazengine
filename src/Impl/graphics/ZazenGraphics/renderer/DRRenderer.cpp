#include "DRRenderer.h"

#include "SkyBox.h"

#include "../Util/GLUtils.h"
#include "../ZazenGraphics.h"
#include "../Geometry/GeometryFactory.h"
#include "../Program/ProgramManagement.h"
#include "../Program/UniformManagement.h"

#include "../context/RenderingContext.h"

#include <GL/glew.h>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <algorithm>

using namespace std;

DRRenderer::DRRenderer()
{
	this->m_gBufferFbo = NULL;
	this->m_intermediateDepthFB = NULL;

	this->m_depthCopy = NULL;

	this->m_progGeomStage = NULL;
	this->m_progSkyBox = NULL;
	this->m_progLightingStage = NULL;
	this->m_progLightingNoShadowStage = NULL;
	this->m_progShadowMappingPlanar = NULL;
	this->m_progShadowMappingCubeSinglePass = NULL;
	this->m_progShadowMappingCubeMultiPass = NULL;
	this->m_progTransparency = NULL;
	this->m_progBlendTransparency = NULL;

	this->m_transformsBlock = NULL;
	this->m_cameraBlock = NULL;
	this->m_lightBlock = NULL;
	this->m_materialBlock = NULL;
	this->m_transparentMaterialBlock = NULL;
	this->m_screenRenderingBoundaryBlock = NULL;

	this->m_fullScreenQuad = NULL;

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
DRRenderer::initialize()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Renderer..." );

	glEnable( GL_DEPTH_TEST );
	
	// Cull triangles which normal is not towards the camera
	glEnable( GL_CULL_FACE );

	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );

	if ( false == this->initFBOs() )
	{
		return false;
	}

	if ( false == this->initGeomStage() )
	{
		return false;
	}

	if ( false == this->initLightingStage() )
	{
		return false;
	}

	if ( false == this->initShadowMapping() )
	{
		return false;
	}

	if ( false == this->initTransparency() )
	{
		return false;
	}

	if ( false == this->initUniformBlocks() )
	{
		return false;
	}

	this->initializeStaticData();

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Renderer finished" );

	return true;
}

bool
DRRenderer::shutdown()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Shutting down Deferred Renderer..." );

	Program::unuse();

	FrameBufferObject::destroy( this->m_intermediateDepthFB );

	// cleaning up framebuffer
	FrameBufferObject::destroy( this->m_gBufferFbo );
	
	// deleting shadow-map pool
	RenderTarget::cleanup();

	ZazenGraphics::getInstance().getLogger().logInfo( "Shutting down Deferred Renderer finished" );

	return true;
}

bool
DRRenderer::initFBOs()
{
	if ( false == this->initGBuffer() )
	{
		return false;
	}

	if ( false == this->initIntermediateDepthBuffer() )
	{
		return false;
	}

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
	
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );
	glClearColor( 0.0, 0.0, 0.0, 1.0 );

	// render-target at index 0: diffuse color
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 1: normals
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 2: positions in viewing-coords (Eye-Space)
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 3: tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 4: bi-tangents
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 5: intermediate lighting-result
	if ( false == this->createMrtBuffer( RenderTarget::RT_COLOR, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// render-target at index 6: depth-buffer
	if ( false == this->createMrtBuffer( RenderTarget::RT_DEPTH, this->m_gBufferFbo ) )		
	{
		return false;
	}

	// IMPORTANT: don't check status until all necessary buffers are created
	// and attached to the according fbo (e.g. after each attach) otherwise
	// it will be incomplete at some point and will fail.
	// check status of FBO AFTER all buffers are set-up and attached
	if ( false == FrameBufferObject::checkStatus() )
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
DRRenderer::initIntermediateDepthBuffer()
{
	// shadow-mapping is done by rendering the depth of the current processing light into an own FBO
	this->m_intermediateDepthFB = FrameBufferObject::create();
	if ( NULL == this->m_intermediateDepthFB )
	{
		return false;
	}

	// bind the shadow FBO to init stuff
	if ( false == this->m_intermediateDepthFB->bind() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initIntermediateDepthBuffer: coulnd't bind shadow FBO - exit" );
		return false;
	}

	// IMPORTANT: disable drawing&reading from this fbo is important, otherwise will fail as incomplete with depth-only attachment
	// no drawing & reading in shadow-fbo, set initial, fbo will keep this state, no need to set it every bind
	this->m_intermediateDepthFB->drawNone();

	// back to default frame-buffer
	if ( false == this->m_intermediateDepthFB->unbind() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initIntermediateDepthBuffer: coulnd't unbind shadow FBO - exit" );
		return false;
	}

	return true;
}

bool
DRRenderer::createMrtBuffer( RenderTarget::RenderTargetType targetType, FrameBufferObject* fbo )
{
	RenderTarget* renderTarget = RenderTarget::create( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight(), targetType );
	if ( NULL == renderTarget )
	{
		return false;
	}

	if ( false == fbo->attachTarget( renderTarget ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::initGeomStage()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Geometry-Stage..." );

	this->m_progGeomStage = ProgramManagement::get( "GeometryStageProgramm" );
	if ( 0 == this->m_progGeomStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't get static geometry program - exit" );
		return false;
	}

	this->m_progSkyBox = ProgramManagement::get( "SkyBoxProgram" );
	if ( 0 == this->m_progSkyBox )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initGeomStage: coulnd't create program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Geometry-Stage finished" );

	return true;
}

bool
DRRenderer::initLightingStage()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Lighting-Stage..." );

	this->m_progLightingStage = ProgramManagement::get( "LightingStageProgramm" );
	if ( 0 == this->m_progLightingStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create program - exit" );
		return false;
	}

	this->m_progLightingNoShadowStage = ProgramManagement::get( "LightingNoShadowStageProgramm" );
	if ( 0 == this->m_progLightingNoShadowStage )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initLightingStage: coulnd't create program - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Lighting-Stage finished" );

	return true;
}

bool
DRRenderer::initShadowMapping()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Shadow-Mapping..." );

	this->m_progShadowMappingPlanar = ProgramManagement::get( "ShadowingStagePlanarProgramm" );
	if ( 0 == this->m_progShadowMappingPlanar )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initShadowMapping: coulnd't create planar-shadow program - exit" );
		return false;
	}

	this->m_progShadowMappingCubeSinglePass = ProgramManagement::get( "ShadowingStageCubeSinglePassProgramm" );
	this->m_progShadowMappingCubeMultiPass = ProgramManagement::get( "ShadowingStageCubeMultiPassProgramm" );
	
	if ( NULL == this->m_progShadowMappingCubeSinglePass && NULL == this->m_progShadowMappingCubeMultiPass )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initShadowMapping: no cube-shadow-mapping program found - exit" );
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Shadow-Mapping finished" );

	return true;
}

bool
DRRenderer::initTransparency()
{
	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Transparency-Stage..." );

	this->m_progTransparency = ProgramManagement::get( "TransparencyProgram" );
	if ( 0 == this->m_progTransparency )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initTransparency: coulnd't create program - exit" );
		return false;
	}

	this->m_progBlendTransparency = ProgramManagement::get( "BlendTransparencyProgram" );
	if ( 0 == this->m_progBlendTransparency )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initTransparency: coulnd't create program - exit" );
		return false;
	}
	
	this->m_fullScreenQuad = GeometryFactory::getRef().createQuad( ( float ) RenderingContext::getRef().getWidth(), ( float ) RenderingContext::getRef().getHeight() );
	if ( NULL == this->m_fullScreenQuad )
	{
		return false;
	}

	this->m_depthCopy = RenderTarget::create( RenderingContext::getRef().getWidth(), RenderingContext::getRef().getHeight(), RenderTarget::RT_SHADOW_PLANAR );
	if ( NULL == this->m_depthCopy )
	{
		return false;
	}

	ZazenGraphics::getInstance().getLogger().logInfo( "Initializing Deferred Rendering Transparency-Stage finished" );

	return true;
}

bool
DRRenderer::initUniformBlocks()
{
	this->m_transformsBlock = UniformManagement::getBlock( "TransformUniforms" );
	if ( NULL == this->m_transformsBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find transforms uniform-block - exit" );
		return false;
	}

	this->m_cameraBlock = UniformManagement::getBlock( "CameraUniforms" );
	if ( 0 == this->m_cameraBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find camera uniform-block - exit" );
		return false;
	}

	this->m_lightBlock = UniformManagement::getBlock( "LightUniforms" );
	if ( 0 == this->m_lightBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find light uniform-block - exit" );
		return false;
	}

	this->m_materialBlock = UniformManagement::getBlock( "MaterialUniforms" );
	if ( 0 == this->m_materialBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find material uniform-block - exit" );
		return false;
	}

	this->m_transparentMaterialBlock = UniformManagement::getBlock( "TransparentMaterialUniforms" );
	if ( 0 == this->m_transparentMaterialBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find transparent material uniform-block - exit" );
		return false;
	}

	this->m_screenRenderingBoundaryBlock = UniformManagement::getBlock( "ScreenRenderingBoundaryUniforms" );
	if ( 0 == this->m_screenRenderingBoundaryBlock )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: couldn't find screen-rendering boundary uniform-block - exit" );
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
	if ( false == this->m_transformsBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding transform uniform-block failed - exit" );
		return false;
	}

	if ( false == this->m_cameraBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding camera uniform-block failed - exit" );
		return false;
	}

	if ( false == this->m_lightBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding light uniform-block failed - exit" );
		return false;
	}

	if ( false == this->m_materialBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding material uniform-block failed - exit" );
		return false;
	}

	if ( false == this->m_transparentMaterialBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding transparent material uniform-block failed - exit" );
		return false;
	}

	if ( false == this->m_screenRenderingBoundaryBlock->bindBase() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::initUniformBlocks: binding screen-rendering boundary uniform-block failed - exit" );
		return false;
	}

	return true;
}

void 
DRRenderer::initializeStaticData()
{
	// insertion-order is very important: first x, then y and last z
	this->m_cubeViewDirections.clear();
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 1, 0, 0 ), glm::vec3( 0, -1, 0 ) ) );		// POS X
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( -1, 0, 0 ), glm::vec3( 0, -1, 0 ) ) );	// NEG X
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 1, 0 ), glm::vec3( 0, 0, 1 ) ) );		// POS Y
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, -1, 0 ), glm::vec3( 0, 0, -1 ) ) ); 	// NEG Y
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 0, 1 ), glm::vec3( 0, -1, 0 ) ) );		// POS Z 
	this->m_cubeViewDirections.push_back( glm::lookAt( glm::vec3( 0 ), glm::vec3( 0, 0, -1 ),glm::vec3( 0, -1, 0 ) ) );		// NEG Z

	this->m_gBufferDrawBufferIndices.clear();
	this->m_gBufferDrawBufferIndices.push_back( 0 );	// diffuse
	this->m_gBufferDrawBufferIndices.push_back( 1 );	// normal
	this->m_gBufferDrawBufferIndices.push_back( 2 );	// position
	this->m_gBufferDrawBufferIndices.push_back( 3 );	// tangents
	this->m_gBufferDrawBufferIndices.push_back( 4 );	// bi-tangents
}

bool
DRRenderer::renderFrame( std::list<ZazenGraphicsEntity*>& entities )
{
	// iterate over all cameras in scene
	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;

		if ( entity->getCamera() )
		{
			this->m_mainCamera = entity->getCamera();
			
			// culling is done outside renderer in special visibility-detection, renderer just issues commands to GPU

			this->preProcessTransparency( entities );

			if ( false == this->doGeometryStage( entities ) )
			{
				return false;
			}

			if ( false == this->doLightingStage( entities ) )
			{
				return false;
			}

			if ( false == this->doTransparencyStage( entities ) )
			{
				return false;
			}
		}
	}
	
	// check once per frame for ALL errors regardless if we are in _DEBUG or not
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderFrame: pending OpenGL-Errors - run the DEBUG-build to narrow and determine the call responsible for the error - exit" );
		return false;
	}

	return true;
}

void
DRRenderer::preProcessTransparency( std::list<ZazenGraphicsEntity*>& entities )
{
	this->m_transparentEntities.clear();

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;

		if ( entity->getMaterial() )
		{
			if ( Material::MATERIAL_TRANSPARENT == entity->getMaterial()->getType() )
			{
				// need to relcalculate distance from viewer for depth-sorting
				entity->recalculateDistance( this->m_mainCamera->getViewMatrix() );

				this->m_transparentEntities.push_back( entity );
			}
		}
	}

	// do depth-sorting
	std::sort( this->m_transparentEntities.begin(), this->m_transparentEntities.end(), DRRenderer::depthSortingFunc );
}

bool
DRRenderer::doGeometryStage( std::list<ZazenGraphicsEntity*>& entities )
{
	// render to g-buffer FBO
	if ( false == this->m_gBufferFbo->bind() )
	{
		return false;
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_mainCamera->restoreViewport();

	// set alpha to 0.0 for skybox/background -> will not run through the whole lighting-shader but just pass through background color
	glClearColor( 0.0, 0.0, 0.0, 0.0 );

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
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::doGeometryStage: using program failed - exit" );
		return false;
	}

	// enable rendering to all render-targets in geometry-stage
	if ( false == this->m_gBufferFbo->drawBuffers( this->m_gBufferDrawBufferIndices ) )
	{
		return false;
	}

	// check status of FBO, IMPORANT: not before, would have failed
	CHECK_FRAMEBUFFER_DEBUG

	// update all parameters of the camera to render geometry-stage
	this->updateCameraBlock( this->m_mainCamera );

	// draw all geometry from cameras viewpoint AND apply materials but ignore transparent material
	if ( false == this->renderEntities( this->m_mainCamera, entities, this->m_progGeomStage, true, false ) )
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
	if ( false == SkyBox::isPresent() )
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
	CHECK_FRAMEBUFFER_DEBUG
		
	// sky-box rendering uses its own program
	if ( false == this->m_progSkyBox->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderSkyBox: using program failed - exit" );
		return false;
	}

	// TODO move into skybox
	// NOTE: need to bind cube-map to unit 8 because unit 0 is already occupied by 2D-textures during light-rendering - it is not allowed to bind different types of textures to the same unit
	this->m_progSkyBox->setUniformInt( "SkyBoxCubeMap", 8 );

	// render the geometry
	SkyBox::getRef().render( *this->m_mainCamera, this->m_cameraBlock, this->m_transformsBlock );

	return true;
}

bool
DRRenderer::doLightingStage( std::list<ZazenGraphicsEntity*>& entities )
{
	// no transparent objects in scene, render lighting to default framebuffer
	if ( 0 == this->m_transparentEntities.size() )
	{
		// clear frame-buffer
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}

	// render the contribution of each light-entiy to the scene
	std::list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entiy = *iter++;
		Light* light = entiy->getLight();
		if ( light )
		{
			if ( false == this->renderLight( entities, light ) )
			{
				return false;
			}
		}
	}

	if ( 0 < this->m_transparentEntities.size() )
	{
		this->m_gBufferFbo->unbind();
	}

	return true;
}

bool
DRRenderer::renderLight( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// different programs could be active according whether light is shadow-caster or not
	// IMPORANT: unbinding textures will fail when shadowmap is not used within program due to no shadow-mapping
	Program* activeLightingProgram = NULL;

	// light is shadow-caster: render shadow map for this light
	if ( light->isShadowCaster() )
	{
		if ( false == this->renderShadowMap( entities, light ) )
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

	// if there are some transparent objects in scene, render lighting result to intermediate 
	// 5th g-buffer target because needed as background for transparency blending
	if ( 0 < this->m_transparentEntities.size() )
	{
		// render to g-buffer
		if ( false == this->m_gBufferFbo->bind() )
		{
			return false;
		}

		// enable rendering to 5th target
		if ( false == this->m_gBufferFbo->drawBuffer( 5 ) )
		{
			return false;
		}

		// check status of FBO, IMPORANT: not before, would have failed
		CHECK_FRAMEBUFFER_DEBUG
	}

	// IMPORTANT: need to re-set the viewport for each FBO
	// could have changed due to shadow-map or other rendering happend in the frame before
	this->m_mainCamera->restoreViewport();

	// activate lighting-stage shader
	if ( false == activeLightingProgram->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderLight: using program failed - exit" );
		return false;
	}

	// OPTIMIZE: no need to do for every light!
	// lighting stage program need all buffers of g-buffer bound as textures
	if ( false == this->m_gBufferFbo->bindTargets( this->m_gBufferDrawBufferIndices ) )
	{
		return false;
	}

	// tell lighting program that diffusemap is bound to texture-unit 0
	activeLightingProgram->setUniformInt( "DiffuseMap", 0 );
	// tell lighting program that normalmap is bound to texture-unit 1
	activeLightingProgram->setUniformInt( "NormalMap", 1 );
	// tell lighting program that generic map is bound to texture-unit 2
	activeLightingProgram->setUniformInt( "PositionMap", 2 );
	// tell lighting program that tangents-map of scene is bound to texture-unit 3 
	activeLightingProgram->setUniformInt( "TangentMap", 3 );
	// tell lighting program that bi-tangents-map of scene is bound to texture-unit 4 
	activeLightingProgram->setUniformInt( "BiTangentMap", 4 );

	// activate the corresponding subroutines for the light-type
	if ( Light::LightType::DIRECTIONAL == light->getType() )
	{
		activeLightingProgram->activateSubroutine( "directionalLight", Shader::FRAGMENT_SHADER );
	}
	else if ( Light::LightType::SPOT == light->getType() )
	{
		activeLightingProgram->activateSubroutine( "spotLight", Shader::FRAGMENT_SHADER );
	}
	else if ( Light::LightType::POINT == light->getType() )
	{
		activeLightingProgram->activateSubroutine( "pointLight", Shader::FRAGMENT_SHADER );
	}

	// bind shadow-map when light is shadow-caster
	if ( light->isShadowCaster() )
	{
		// WARNING: you can't bind two different texture-types to the same unit in the same program - will result in INVALID OPERATION on draw-call
		// thus we need to bind the shadow cube-map to a different unit than the normal 2d shadow-map
		int textureUnit = 7;

		if ( Light::LightType::POINT == light->getType() )
		{
			textureUnit = 8;

			// tell program that the shadowmap of point-light will be available at texture unit 7
			activeLightingProgram->setUniformInt( "ShadowCubeMap", textureUnit );
		}
		else
		{
			// tell program that the shadowmap of spot/directional-light will be available at texture unit 6
			activeLightingProgram->setUniformInt( "ShadowPlanarMap", textureUnit );
		}

		if ( false == light->getShadowMap()->bind( textureUnit ) )
		{
			return false;
		}
	}

	// update configuration of the current light to its uniform-block
	if ( false == this->updateLightBlock( light ) )
	{
		return false;
	}

	// need to re-set the configuration of the camera uniform-block to the main-camera because need the information within the lighting-shader
	if ( false == this->updateCameraBlock( this->m_mainCamera ) )
	{
		return false;
	}

	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_mainCamera->createOrthoProj( true, true );
	if ( false == this->updateScreenRenderingBlock( orthoMat ) )
	{
		return false;
	}
	
	// blend lighting-results 
	glEnable( GL_BLEND );
	// need additive-blending
	glBlendEquation( GL_FUNC_ADD );
	// TODO: do some research why this blending-function actually works (it does!)
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_COLOR );

	// disable writing to depth: light-boundaries should not update depth
	// we also need to READ from depth so no update 
	glDepthMask( GL_FALSE );

	// render light boundary (for now only full screen-quad)
	light->getBoundingMesh()->render();
	
	// enable depth-writing again
	glDepthMask( GL_TRUE );

	// turn off blending otherwise would lead to artifacts because everything would be blended (depth maps,...)
	glDisable( GL_BLEND );

	return true;
}

bool
DRRenderer::renderShadowMap( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// update the camera for rendering the shadow-cube - the light is the camera because we render the scene from the perspective of the camera
	this->updateCameraBlock( light );

	// use shadow-mapping fbo to render depth of light view-point to
	if ( false == this->m_intermediateDepthFB->bind() )
	{
		return false;
	}

	// point-light has a cube-shadowmap
	if ( Light::LightType::POINT == light->getType() )
	{
		if ( false == this->renderShadowCube( entities, light ) )
		{
			return false;
		}
	}
	// directional- and spot-light have a planar (2D) shadow-map 
	else
	{
		if ( false == this->renderShadowPlanar( entities, light ) )
		{
			return false;
		}
	}

	// back to default framebuffer
	if ( false == this->m_intermediateDepthFB->unbind() )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowPlanar( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	if ( false == this->m_intermediateDepthFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	if ( false == this->m_progShadowMappingPlanar->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowMap: using planar program failed - exit" );
		return false;
	}

	if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingPlanar ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowCube( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	// single-pass program for shadow-mapping is in use
	// uses geometry-shader to render to layers of the cube-map
	if ( this->m_progShadowMappingCubeSinglePass )
	{
		if ( false == this->renderShadowCubeSinglePass( entities, light ) )
		{
			return false;
		}
	}
	// multi-pass program for shadow-mapping is in use
	// do a pass for each cube-layer
	else
	{
		if ( false == this->renderShadowCubeMultiPass( entities, light ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderShadowCubeSinglePass( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	std::vector<glm::mat4> cubeMVPTransforms( 6 );
	glm::vec3 lightPosWorld = light->getPosition();
	glm::mat4 invLightPosTransf = glm::translate( -lightPosWorld );
		
	// calculate model-view-projection matrices for each cube-face
	for ( unsigned int i = 0; i < 6; ++i )
	{
		cubeMVPTransforms[ i ] = light->getProjMatrix() * this->m_cubeViewDirections[ i ] * invLightPosTransf;
	}

	if ( false == this->m_progShadowMappingCubeSinglePass->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowCubeSinglePass: using cube single-pass program failed - exit" );
		return false;
	}

	// upload model-view-projection transformations for each face (used in geometry-shader)
	this->m_progShadowMappingCubeSinglePass->setUniformMatrices( "u_cubeMVPTransforms[0]", cubeMVPTransforms );

	// attach cube-map faces to frame-buffer object
	if ( false == this->m_intermediateDepthFB->attachTargetTemp( light->getShadowMap() ) )
	{
		return false;
	}

	// do single shadow-pass - geometry-shader will render to each layer of the cube-map
	if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingCubeSinglePass ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderShadowCubeMultiPass( std::list<ZazenGraphicsEntity*>& entities, Light* light )
{
	std::vector<glm::mat4> cubeViewTransforms( 6 );
	glm::vec3 lightPosWorld = light->getPosition();
	glm::mat4 invLightPosTransf = glm::translate( -lightPosWorld );
		
	// calculate model-view matrices for each cube-face - leave out projection, will be applied in shader!
	for ( unsigned int i = 0; i < 6; ++i )
	{
		cubeViewTransforms[ i ] = this->m_cubeViewDirections[ i ] * invLightPosTransf;
	}

	if ( false == this->m_progShadowMappingCubeMultiPass->use() )
	{
		ZazenGraphics::getInstance().getLogger().logError( "DRRenderer::renderShadowCubeMultiPass: using cube multi-pass program failed - exit" );
		return false;
	}

	// do 6 passes, one for each cube-map face
	for ( unsigned int face = 0; face < 6; ++face )
	{
		if ( false == this->m_cameraBlock->bindBuffer() )
		{
			return false;
		}

		// update viewing-matrix for the according cube-face
		this->m_cameraBlock->updateField( "CameraUniforms.viewMatrix", cubeViewTransforms[ face ] );

		// attach cube-map faces to frame-buffer object
		if ( false == this->m_intermediateDepthFB->attachTargetTempCubeFace( light->getShadowMap(), face ) )
		{
			return false;
		}

		if ( false == this->renderShadowPass( entities, light, this->m_progShadowMappingCubeMultiPass ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderShadowPass( list<ZazenGraphicsEntity*>& entities, Light* light, Program* currentShadowProgram )
{
	// check status now
	// IMPORANT: don't check too early
	CHECK_FRAMEBUFFER_DEBUG

	// clear bound buffer
	// TODO: GL_DEPTH_BUFFER_BIT should be enough if really only the depth is rendered
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// IMPORTANT: need to set the viewport for each shadow-map, because resolution can be different for each
	light->restoreViewport();

	// render scene from view of camera - don't apply material, we need only depth, render transparent materials
	if ( false == this->renderEntities( light, entities, currentShadowProgram, false, true ) )
	{
		return false;
	}

	// render scene from view of camera - don't apply material, we need only depth, render opaque materials
	if ( false == this->renderEntities( light, entities, currentShadowProgram, false, false ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::doTransparencyStage( std::list<ZazenGraphicsEntity*>& entities )
{
	unsigned int combinationTarget = 1;
	unsigned int backgroundIndex = 5;

	for ( unsigned int i = 0; i < this->m_transparentEntities.size(); i++ )
	{
		ZazenGraphicsEntity* entity = this->m_transparentEntities[ i ];

		if ( false == this->renderTransparentInstance( entity, backgroundIndex, combinationTarget,
			i == this->m_transparentEntities.size() - 1 ) )
		{
			return false;
		}

		std::swap( combinationTarget, backgroundIndex );
	}

	return true;
}

bool
DRRenderer::renderTransparentInstance( ZazenGraphicsEntity* entity, unsigned int backgroundIndex, unsigned int combinationTarget, bool lastInstance )
{
	if ( false == this->m_progTransparency->use() )
	{
		return false;
	}

	// bind background to index 2 because DiffuseColor of Material is at index 0 and NormalMap of Material is at index 1
	this->m_gBufferFbo->getAttachedTargets()[ backgroundIndex ]->bind( 2 );
	// bind depth-copy to index 3
	this->m_depthCopy->bind( 3 );

	this->m_progTransparency->setUniformInt( "Background", 2 );
	this->m_progTransparency->setUniformInt( "BackgroundDepth", 3 );

	this->m_gBufferFbo->bind();
	
	// copy depth of g-buffer to target, because we need to access the depth in transparency-rendering
	// to prevent artifacts. at the same time we write depth when rendering transparency so
	// we cannot bind the g-buffer depth => need to copy
	if ( false == this->m_gBufferFbo->copyDepthToTarget( this->m_depthCopy ) )
	{
		return false;
	}

	// transparent objects are always rendered intermediate to g-buffer color target 0
	this->m_gBufferFbo->drawBuffer( 0 );

	// clear buffer 
	glClear( GL_COLOR_BUFFER_BIT );

	if ( false == this->updateCameraBlock( this->m_mainCamera ) )
	{
		return false;
	}

	if ( false == this->renderTransparentEntity( this->m_mainCamera, entity, this->m_progTransparency ) )
	{
		return false;
	}

	// after rendering last instance we will combine to default-framebuffer for final result
	if ( lastInstance )
	{
		// back to default-framebuffer
		this->m_gBufferFbo->unbind();

		// clear default-framebuffer color & depth
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	}
	else 
	{
		// draw combination to new target (ping-pong rendering)
		this->m_gBufferFbo->drawBuffer( combinationTarget );

		glClear( GL_COLOR_BUFFER_BIT );
	}

	if ( false == this->m_progBlendTransparency->use() )
	{
		return false;
	}

	// transparent object was rendered to g-buffer color target 0
	this->m_gBufferFbo->getAttachedTargets()[ 0 ]->bind( 0 );

	this->m_progBlendTransparency->setUniformInt( "Background", 2 );
	this->m_progBlendTransparency->setUniformInt( "Transparent", 0 );
	
	// OPTIMIZE: store in light once, and only update when change
	glm::mat4 orthoMat = this->m_mainCamera->createOrthoProj( true, true );
	if ( false == this->updateScreenRenderingBlock( orthoMat ) )
	{
		return false;
	}

	// disable writing to depth when not last instance because would destroy our depth-buffer
	// when last instance it doesnt matter because we render to screen-buffer
	if ( ! lastInstance )
	{
		glDepthMask( GL_FALSE );
	}

	this->m_fullScreenQuad->render();

	// enable depth-writing again
	if ( ! lastInstance )
	{
		glDepthMask( GL_TRUE );
	}

	return true;
}

bool
DRRenderer::renderEntities( Viewer* viewer, list<ZazenGraphicsEntity*>& entities, Program* currentProgramm, bool applyMaterial, bool renderTransparency )
{
	// bind transform uniform-block to update model-, view & projection transforms
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}

	list<ZazenGraphicsEntity*>::iterator iter = entities.begin();
	while ( iter != entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		Material* material = entity->getMaterial();
		Animation* animation = entity->getAnimation();

		if ( 0 == entity->getMeshNode() )
		{
			continue;
		}

		if ( animation )
		{
			// upload bones for animation
			currentProgramm->setUniformMatrices( "u_bones[0]", animation->getBoneTransforms() );
			currentProgramm->activateSubroutine( "processInputsAnimated", Shader::VERTEX_SHADER );
		}
		else
		{
			// switch subroutine in vertex-shader of geometry-stage because need skinning for animated instances
			currentProgramm->activateSubroutine( "processInputsStatic", Shader::VERTEX_SHADER );
		}

		if ( material )
		{
			if ( ( Material::MATERIAL_TRANSPARENT == material->getType() && ! renderTransparency ) ||
				( Material::MATERIAL_TRANSPARENT != material->getType() && renderTransparency ) )
			{
				continue;
			}

			// activate material only when there is one present & render-pass enforces usage of materials
			if ( applyMaterial )
			{
				// activate material
				if ( false == material->activate( currentProgramm ) )
				{
					return false;
				}

				// back to transform uniform-block because is needed in the actual rendering of geometry (modelview calculation)
				if ( false == this->m_transformsBlock->bindBuffer() )
				{
					return false;
				}
			}
		}

		// calculate model-view matrix once for this entity, hierarchical node-transforms are applied during mesh-rendering
		glm::mat4 modelView = viewer->getViewMatrix() * entity->getModelMatrix();

		// render geometry of this instance
		if ( false == this->renderMeshNode( entity->getMeshNode(), modelView, entity->getModelMatrix() ) )
		{
			return false;
		}
	}

	return true;
}

bool
DRRenderer::renderTransparentEntity( Viewer* viewer, ZazenGraphicsEntity* entity, Program* currentProgramm )
{
	// activate material
	if ( false == entity->getMaterial()->activate( currentProgramm ) )
	{
		return false;
	}

	// back to transform uniform-block because is needed in the actual rendering of geometry (modelview calculation)
	if ( false == this->m_transformsBlock->bindBuffer() )
	{
		return false;
	}
		
	// calculate model-view matrix once for this entity, hierarchical node-transforms are applied during mesh-rendering
	glm::mat4 modelView = viewer->getViewMatrix() * entity->getModelMatrix();

	// render geometry of this instance
	if ( false == this->renderMeshNode( entity->getMeshNode(), modelView, entity->getModelMatrix() ) )
	{
		return false;
	}

	return true;
}

bool
DRRenderer::renderMeshNode( MeshNode* meshNode, const glm::mat4& entityModelViewMatrix, const glm::mat4& entitiyModelMatrix )
{
	// no meshes in this node or any sub-children, nothing to render
	// because we pre-calculate the global-transformation for each mesh-node during loading
	// we can ommit non-meshed nodes and don't need to walk them just because of re-calculating
	// the (always static) hierarchical global-transformations
	if ( ! meshNode->hasMeshes() )
	{
		return true;
	}

	const std::vector<Mesh*>& meshes = meshNode->getMeshes();
	const std::vector<MeshNode*>& children = meshNode->getChildren();
	glm::mat4 localModelViewMatrix = entityModelViewMatrix * meshNode->getGlobalTransform();
	glm::mat4 localModelMatrix = entitiyModelMatrix * meshNode->getGlobalTransform();

	for ( unsigned int i = 0; i < children.size(); i++ )
	{
		MeshNode* child = children[ i ];
		if ( false == this->renderMeshNode( children[ i ], entityModelViewMatrix, entitiyModelMatrix ) )
		{
			return false;
		}
	}

	for ( unsigned int i = 0; i < meshes.size(); i++ )
	{
		Mesh* mesh = meshes[ i ];

		// update model matrix
		this->m_transformsBlock->updateField( "TransformUniforms.modelMatrix", localModelMatrix );
		// update model-view matrix
		this->m_transformsBlock->updateField( "TransformUniforms.modelViewMatrix", localModelViewMatrix );
		
		// THIS EXISTS FOR LEARNING AND INFORMATION REATIONS!
		// IMPORTANT: normal-vectors are transformed different than vertices
		// take the transpose of the inverse modelView or simply reset the translation vector in the modelview-matrix
		// in other words: only the rotations are applied to normals and they are guaranteed to leave
		// normalized normals at unit length. THIS METHOD WORKS ALSO WHEN NO NON UNIFORM SCALING IS APPLIED
		// TODO: scrap this transform because we forbid non-uniform-scaling in our engine for performance reasons
		// glm::mat4 normalModelViewMatrix = glm::transpose( glm::inverse( localModelViewMatrix ) );
		// update model-view matrix for normals
		// this->m_transformsBlock->updateField( "TransformUniforms.normalsModelViewMatrix", normalModelViewMatrix );

		// render geometry
		return mesh->render();
	}

	return true;
}

bool
DRRenderer::depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b )
{
	// depth sorting: distance holds the z-value of the center in view-space
	// the larger the negative values the farther way
	return a->getDistance() < b->getDistance(); 
}

bool
DRRenderer::updateCameraBlock( Viewer* viewer )
{
	glm::vec2 cameraRectangle;
	glm::vec2 cameraNearFar;
	
	cameraRectangle[ 0 ] = ( float ) viewer->getWidth();
	cameraRectangle[ 1 ] = ( float ) viewer->getHeight();

	cameraNearFar[ 0 ] = viewer->getNear();
	cameraNearFar[ 1 ] = viewer->getFar();

	// bind camera uniform-block to update data
	if ( false == this->m_cameraBlock->bindBuffer() )
	{
		return false;
	}

	// upload cameras window size
	this->m_cameraBlock->updateField( "CameraUniforms.window", cameraRectangle );
	// upload cameras near and far distances
	this->m_cameraBlock->updateField( "CameraUniforms.nearFar", cameraNearFar );

	// upload world-orientation of camera ( its model-matrix )
	this->m_cameraBlock->updateField( "CameraUniforms.modelMatrix", viewer->getModelMatrix() );
	// upload view-matrix of camera ( = its inverse model-matrix)
	this->m_cameraBlock->updateField( "CameraUniforms.viewMatrix", viewer->getViewMatrix() );
	// upload projection-matrix of the camera
	this->m_cameraBlock->updateField( "CameraUniforms.projectionMatrix", viewer->getProjMatrix() );
	
	return true;
}

bool
DRRenderer::updateLightBlock( Light* light )
{
	glm::vec2 shadowResolution;
	const glm::vec3& color = light->getColor();
	const glm::vec2& specular = light->getSpecular();
	const glm::vec3& attenuation = light->getAttenuation();

	// shadow-map resolution corresponds to the width and height
	shadowResolution.x = ( float ) light->getWidth();
	shadowResolution.y = ( float )  light->getHeight();
	
	// bind light uniform-block to update data of this light
	if ( false == this->m_lightBlock->bindBuffer() )
	{
		return false;
	}

	this->m_lightBlock->updateField( "LightUniforms.shadowResolution", shadowResolution );
	this->m_lightBlock->updateField( "LightUniforms.color", color );
	this->m_lightBlock->updateField( "LightUniforms.specular", specular );
	
	// upload light-model matrix = orientation of the light in the world
	this->m_lightBlock->updateField( "LightUniforms.modelMatrix", light->getModelMatrix() );

	if ( Light::LightType::POINT == light->getType() )
	{
		glm::vec2 nearFar;
		nearFar.x = light->getNear();
		nearFar.y = light->getFar();

		this->m_lightBlock->updateField( "LightUniforms.nearFar", nearFar );
		this->m_lightBlock->updateField( "LightUniforms.attenuation", attenuation );
	}
	else
	{
		if ( Light::LightType::SPOT == light->getType() )
		{
			glm::vec2 spot( 1.0 );
			spot.x = ( float ) cos( light->getFov() / 2.0 );
			spot.y = 60.0;

			// TODO: make spot-exponent configurable

			this->m_lightBlock->updateField( "LightUniforms.attenuation", attenuation );
			this->m_lightBlock->updateField( "LightUniforms.spot", spot );
		}

		// update unit-cube matrix only when its NOT a point light AND shadow-caster
		// this unit-cube matrix is only needed in case of planar shadow rendering
		if ( light->isShadowCaster() )
		{
			glm::mat4 lightSpaceUnit = this->m_unitCubeMatrix * light->getVPMatrix();
			this->m_lightBlock->updateField( "LightUniforms.spaceUniformMatrix", lightSpaceUnit );
		}
	}

	return true;
}

bool
DRRenderer::updateScreenRenderingBlock( const glm::mat4& projection )
{
	if ( false == this->m_screenRenderingBoundaryBlock->bindBuffer() )
	{
		return false;
	}

	this->m_screenRenderingBoundaryBlock->updateField( "ScreenRenderingBoundaryUniforms.projectionMatrix", projection );

	return true;
}
