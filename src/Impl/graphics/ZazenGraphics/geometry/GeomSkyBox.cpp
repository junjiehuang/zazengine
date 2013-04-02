/*
 *  spacebox.cpp
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "GeomSkyBox.h"

#include "GeometryFactory.h"

#include "../ZazenGraphics.h"
#include <glm/gtc/type_ptr.hpp>

using namespace std;

#define BOX_SIDE_SIZE 100

GeomSkyBox* GeomSkyBox::instance = NULL;

bool
GeomSkyBox::initialize( const boost::filesystem::path& textureFolder, const std::string& format )
{
	if ( NULL == GeomSkyBox::instance )
	{
		new GeomSkyBox();

		GeomSkyBox::instance->m_frontText = Texture::get( textureFolder.generic_string() + "/front." + format );
		if ( NULL == GeomSkyBox::instance->m_frontText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get front-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_backText = Texture::get( textureFolder.generic_string() + "/back." + format );
		if ( NULL == GeomSkyBox::instance->m_backText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get back-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_leftText = Texture::get( textureFolder.generic_string() + "/left." + format );
		if ( NULL == GeomSkyBox::instance->m_leftText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get left-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_rightText = Texture::get( textureFolder.generic_string() + "/right." + format );
		if ( NULL == GeomSkyBox::instance->m_rightText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get right-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_upText = Texture::get( textureFolder.generic_string() + "/up." + format );
		if ( NULL == GeomSkyBox::instance->m_upText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get up-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_downText = Texture::get( textureFolder.generic_string() + "/down." + format );
		if ( NULL == GeomSkyBox::instance->m_downText )
		{
			cout << "ERROR ... in GeomSkyBox::initialize: couldn't get down-texture" << endl;
			GeomSkyBox::shutdown();
			return false;
		}

		GeomSkyBox::instance->m_frontGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 frontModelMatrix;
		frontModelMatrix[ 3 ][ 2 ] = -BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_frontGeom->setModelMatrix( frontModelMatrix );

		GeomSkyBox::instance->m_backGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 backModelMatrix;
		backModelMatrix[ 3 ][ 2 ] = BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_backGeom->setModelMatrix( backModelMatrix );

		GeomSkyBox::instance->m_leftGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 leftModelMatrix;
		leftModelMatrix[ 2 ][ 0 ] = 1.0f;
		leftModelMatrix[ 3 ][ 0 ] = -BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_leftGeom->setModelMatrix( leftModelMatrix );

		GeomSkyBox::instance->m_rightGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 rightModelMatrix;
		rightModelMatrix[ 2 ][ 0 ] = -1.0f;
		rightModelMatrix[ 3 ][ 0 ] = BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_rightGeom->setModelMatrix( rightModelMatrix );

		GeomSkyBox::instance->m_upGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 upModelMatrix;
		upModelMatrix[ 2 ][ 1 ] = -1.0f;
		upModelMatrix[ 3 ][ 0 ] = BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_upGeom->setModelMatrix( upModelMatrix );

		GeomSkyBox::instance->m_downGeom = GeometryFactory::createQuad( BOX_SIDE_SIZE, BOX_SIDE_SIZE );
		glm::mat4 downModelMatrix;
		downModelMatrix[ 2 ][ 1 ] = 1.0f;
		downModelMatrix[ 3 ][ 0 ] = BOX_SIDE_SIZE;
		GeomSkyBox::instance->m_downGeom->setModelMatrix( downModelMatrix );
	}

	return true;
}

bool
GeomSkyBox::shutdown()
{
	if ( GeomSkyBox::instance )
	{
		delete GeomSkyBox::instance;
	}

	return true;
}

GeomSkyBox::GeomSkyBox()
{
	this->m_frontText = NULL;
	this->m_backText = NULL;
	this->m_leftText = NULL;
	this->m_rightText = NULL;
	this->m_upText = NULL;
	this->m_downText = NULL;

	this->m_frontGeom = NULL;
	this->m_backGeom = NULL;
	this->m_leftGeom = NULL;
	this->m_rightGeom = NULL;
	this->m_upGeom = NULL;
	this->m_downGeom = NULL;

	GeomSkyBox::instance = this;
}

GeomSkyBox::~GeomSkyBox()
{
	GeomSkyBox::instance = NULL;
}

bool
GeomSkyBox::render()
{
	Viewer& cam = ZazenGraphics::getInstance().getCamera();

	// disable depth-writing, sky-box is ALWAYS behind everything else
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_CULL_FACE );

	// force projective-projection in sky-box (camera could have ortho)
	glm::mat4 projMat = ZazenGraphics::getInstance().getCamera().createPerspProj();
	// reset modelview back to origin, to stick with camera, sky-box is so far away that it doesn't move
	glm::mat4 modelViewMat = cam.getViewMatrix();
	modelViewMat[ 3 ][ 0 ] = 0.0f;
	modelViewMat[ 3 ][ 1 ] = 0.0f;
	modelViewMat[ 3 ][ 2 ] = 0.0f;

	// TODO: update transforms uniform block

	// Front Face
	this->m_frontText->bind( 0 );

	this->m_frontGeom->render();
	/////////////////////////

	// Back Face
	this->m_backText->bind( 0 );

	this->m_backGeom->render();
	/////////////////////////

	// Top Face
	this->m_upText->bind( 0 );
	
	this->m_upGeom->render();
	/////////////////////////

	// Bottom Face
	this->m_downText->bind( 0 );

	this->m_downGeom->render();
	/////////////////////////

	// Right face
	this->m_rightText->bind( 0 );

	this->m_rightGeom->render();
	/////////////////////////

	// Left Face
	this->m_leftText->bind( 0 );

	this->m_leftGeom->render();
	/////////////////////////

	// activate z-buffering and face culling
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_CULL_FACE );

	return true;
}
