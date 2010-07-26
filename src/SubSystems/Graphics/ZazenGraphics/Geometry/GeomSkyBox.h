/*
 *  spacebox.h
 *  ZENgine
 *
 *  Created by Jonathan Thaler on 02.05.08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GEOMSPACEBOX_H_
#define _GEOMSPACEBOX_H_

#include "GeomType.h"
#include "../Material/Texture.h"
#include "../Scene/Camera.h"

class GeomSkyBox : public GeomType
{
 public:
	GeomSkyBox( Camera& cam, std::string& textures );
	virtual ~GeomSkyBox();
	
	void render();
	
 private:
	Camera& cam;

	Texture* east;
	Texture* west;
	Texture* up;
	Texture* down;
	Texture* south;
	Texture* north;
};

#endif
