/*
 * Material.cpp
 *
 *  Created on: Feb 25, 2011
 *      Author: jonathan
 */

#include "Material.h"

#include <iostream>

using namespace std;

Material::Material( const std::string& name, MaterialType type )
	: m_name( name ),
	  m_type( type )
{
}

Material::~Material()
{
}
