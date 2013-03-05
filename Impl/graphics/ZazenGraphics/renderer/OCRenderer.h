#ifndef OCRENDERER_H_
#define OCRENDERER_H_

#include <list>

#include "Renderer.h"

class OCRenderer : public Renderer
{
 public:
	OCRenderer(Camera& camera, std::string&);
	virtual ~OCRenderer();

	 // renders this list of geominstances which must be in front-to-back order
	 bool renderFrame( std::list<Instance*>& instances );

 private:
	 bool parentIntersectingFrustum;

	  typedef struct {
		 GLuint id;
		 std::list<Instance*> items;	// used for multiqueries: when size > 1 => multiqueries, otherwise single-query
	 } QueryItem;

	 std::list<Instance*> distanceQueue;

	 std::list<Instance*> iQueries;
	 std::list<Instance*> vQueries;

	 std::list<QueryItem> queries;

	 bool firstQueryFinished();
	 void traverseInstance( Instance* );
	 void handleReturnedQuery( QueryItem );
	 void pullUpVisibility( Instance* );
	 void queryIndividualNodes(QueryItem);
	 void issueQuery( Instance* );
	 void issueMultiQueries();
	 void issuePreviouslyInvisibleNode( Instance* );
};

#endif /*OCRENDERER_H_*/
