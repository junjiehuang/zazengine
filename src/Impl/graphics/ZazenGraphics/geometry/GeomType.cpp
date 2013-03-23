#include "GeomType.h"

GeomType::GeomType()
{
	this->parent = 0;
	this->bbVBO = 0;
}

GeomType::~GeomType()
{
}

void
GeomType::compareBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	for ( int i = 0; i < 3; i++ )
	{
		if ( bbMax[ i ] > this->bbMax[ i ] )
		{
			this->bbMax[ i ] = bbMax[ i ];
		}
		else if ( bbMin[ i ] < this->bbMin[ i ] )
		{
			this->bbMin[ i ] = bbMin[ i ];
		}
	}

	this->setBB( this->bbMin, this->bbMax );
}
 
void
GeomType::setBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	this->bbMin = bbMin;
	this->bbMax = bbMax;
	
	for ( int i = 0; i < 3; i++ )
	{
		this->center[ i ] = this->bbMin[ i ] + ( ( this->bbMax[ i ] - this->bbMin[ i ] ) / 2 );
	}
}

void
GeomType::renderBB()
{
	/*
	if (this->bbVBO == 0) {
		float bbFaces[24][3];
		memset(bbFaces, 0, sizeof(bbFaces));
		
		// LEFT
		bbFaces[0][0] = this->bbMin[0];
		bbFaces[0][1] = this->bbMin[1];
		bbFaces[0][2] = this->bbMin[2];
		
		bbFaces[1][0] = this->bbMin[0];
		bbFaces[1][1] = this->bbMin[1];
		bbFaces[1][2] = this->bbMax[2];
		
		bbFaces[2][0] = this->bbMin[0];
		bbFaces[2][1] = this->bbMax[1];
		bbFaces[2][2] = this->bbMax[2];

		bbFaces[3][0] = this->bbMin[0];
		bbFaces[3][1] = this->bbMax[1];
		bbFaces[3][2] = this->bbMin[2];

		// RIGHT
		bbFaces[4][0] = this->bbMax[0];
		bbFaces[4][1] = this->bbMin[1];
		bbFaces[4][2] = this->bbMin[2];
		
		bbFaces[5][0] = this->bbMax[0];
		bbFaces[5][1] = this->bbMax[1];
		bbFaces[5][2] = this->bbMin[2];
		
		bbFaces[6][0] = this->bbMax[0];
		bbFaces[6][1] = this->bbMax[1];
		bbFaces[6][2] = this->bbMax[2];

		bbFaces[7][0] = this->bbMax[0];
		bbFaces[7][1] = this->bbMin[1];
		bbFaces[7][2] = this->bbMax[2];

		// front
		bbFaces[8][0] = this->bbMin[0];
		bbFaces[8][1] = this->bbMin[1];
		bbFaces[8][2] = this->bbMax[2];
		
		bbFaces[9][0] = this->bbMax[0];
		bbFaces[9][1] = this->bbMin[1];
		bbFaces[9][2] = this->bbMax[2];
		
		bbFaces[10][0] = this->bbMax[0];
		bbFaces[10][1] = this->bbMax[1];
		bbFaces[10][2] = this->bbMax[2];

		bbFaces[11][0] = this->bbMin[0];
		bbFaces[11][1] = this->bbMax[1];
		bbFaces[11][2] = this->bbMax[2];

		// back
		bbFaces[12][0] = this->bbMax[0];
		bbFaces[12][1] = this->bbMin[1];
		bbFaces[12][2] = this->bbMin[2];
		
		bbFaces[13][0] = this->bbMin[0];
		bbFaces[13][1] = this->bbMin[1];
		bbFaces[13][2] = this->bbMin[2];
		
		bbFaces[14][0] = this->bbMin[0];
		bbFaces[14][1] = this->bbMax[1];
		bbFaces[14][2] = this->bbMin[2];

		bbFaces[15][0] = this->bbMax[0];
		bbFaces[15][1] = this->bbMax[1];
		bbFaces[15][2] = this->bbMin[2];

		// top
		bbFaces[16][0] = this->bbMin[0];
		bbFaces[16][1] = this->bbMax[1];
		bbFaces[16][2] = this->bbMax[2];
		
		bbFaces[17][0] = this->bbMax[0];
		bbFaces[17][1] = this->bbMax[1];
		bbFaces[17][2] = this->bbMax[2];
		
		bbFaces[18][0] = this->bbMax[0];
		bbFaces[18][1] = this->bbMax[1];
		bbFaces[18][2] = this->bbMin[2];

		bbFaces[19][0] = this->bbMin[0];
		bbFaces[19][1] = this->bbMax[1];
		bbFaces[19][2] = this->bbMin[2];
				
		// bottom
		bbFaces[20][0] = this->bbMin[0];
		bbFaces[20][1] = this->bbMin[1];
		bbFaces[20][2] = this->bbMin[2];
		
		bbFaces[21][0] = this->bbMax[0];
		bbFaces[21][1] = this->bbMin[1];
		bbFaces[21][2] = this->bbMin[2];
		
		bbFaces[22][0] = this->bbMax[0];
		bbFaces[22][1] = this->bbMin[1];
		bbFaces[22][2] = this->bbMax[2];

		bbFaces[23][0] = this->bbMin[0];
		bbFaces[23][1] = this->bbMin[1];
		bbFaces[23][2] = this->bbMax[2];

		glGenBuffers(1, &this->bbVBO);
		glBindBuffer(GL_ARRAY_BUFFER, this->bbVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(bbFaces), bbFaces, GL_STATIC_DRAW);
	}

	glEnableClientState(GL_VERTEX_ARRAY);
 
	glBindBuffer(GL_ARRAY_BUFFER, this->bbVBO);
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glDrawArrays(GL_QUADS, 0, 24);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	*/
	
	glBegin(GL_QUADS);		
		// TOP
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
		
		// BOTTOM
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
	
		
		// LEFT
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
		
		// RIGHT
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
		
		// FRONT
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		
		// BACK
		glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
		glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
		glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
	glEnd();
}

bool
GeomType::render()
{
	//if (ZENgine::getInstance().drawBoundingBox()) {
		glBegin(GL_LINES);
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
		
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
		
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
		
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMin[2]);
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
		
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
		
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMax[2]);
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
		
			glVertex3f(this->bbMin[0], this->bbMin[1], this->bbMax[2]);
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
		
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMax[2]);
			glVertex3f(this->bbMax[0], this->bbMin[1], this->bbMin[2]);
		
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
			glVertex3f(this->bbMax[0], this->bbMax[1], this->bbMin[2]);
		
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMin[2]);
			glVertex3f(this->bbMin[0], this->bbMax[1], this->bbMax[2]);
		glEnd();
		
		this->renderBB();
	//}

	return true;
}
