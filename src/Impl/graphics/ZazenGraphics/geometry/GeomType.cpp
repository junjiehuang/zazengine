#include "GeomType.h"

GeomType::GeomType()
{
}

GeomType::~GeomType()
{
	for ( unsigned int i = 0; i < this->m_children.size(); i++ )
	{
		delete this->m_children[ i ];
	}
}

void
GeomType::compareBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	for ( int i = 0; i < 3; i++ )
	{
		if ( bbMax[ i ] > this->m_bbMax[ i ] )
		{
			this->m_bbMax[ i ] = bbMax[ i ];
		}
		else if ( bbMin[ i ] < this->m_bbMin[ i ] )
		{
			this->m_bbMin[ i ] = bbMin[ i ];
		}
	}

	this->setBB( this->m_bbMin, this->m_bbMax );
}
 
void
GeomType::setBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	this->m_bbMin = bbMin;
	this->m_bbMax = bbMax;
	
	for ( int i = 0; i < 3; i++ )
	{
		this->m_center[ i ] = this->m_bbMin[ i ] + ( ( this->m_bbMax[ i ] - this->m_bbMin[ i ] ) / 2 );
	}
}
