#include <GL/glew.h>
#include "SDL/SDL.h"

#include "OCRenderer.h"

#include "../States/Statemanager.h"
#include "../States/Occlusionquerystate.h"

#include <iostream>
#include <algorithm>
#include <assert.h>

#define INVISIBLE_BATCH_SIZE 50
#define VISIBLE_PIXEL_THRESH 50

using namespace std;

/* TODO:
 * problem: the visibility queries of already rendered nodes are returning when waiting for a previously issued multiquery
 * then they are detected to be visible and are rendered again: COULD RESULT IN DOUBLING OF RENDERING EACH OBJECT!!
 */

OCRenderer::OCRenderer(Camera& camera, std::string& skyBoxFolder)
	: Renderer(camera, skyBoxFolder)
{
	StateManager::getInstance().registerState(new OcclusionQueryState());
}

OCRenderer::~OCRenderer()
{
}

bool
OCRenderer::renderFrame( std::list<Instance*>& instances )
{
	this->renderedFaces = 0;
	this->renderedInstances = 0;

	this->culledInstances = 0;
	this->occludedInstances = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadMatrixf(this->camera.modelView.data);

	root->lastFrame = this->frame - 1;
	root->visible = true;

	this->distanceQueue.push_back(root);

#ifdef OC_OUTPUT
	cout << "pushed root..." << endl;
#endif

	while (this->distanceQueue.size() || this->queries.size()) {
#ifdef OC_OUTPUT
		cout << "still something to do... instances = " << this->distanceQueue.size() << " OR queries = " <<  this->queries.size() << endl;
#endif

		// process the finished queries
		while (this->queries.size()) {
			//	checks if the first query is available
			if (this->firstQueryFinished()) {
#ifdef OC_OUTPUT
			cout << "queries left... first query YES finished" << endl;
#endif
				QueryItem q = this->queries.front();
				this->queries.pop_front();

				this->handleReturnedQuery(q);

			} else {
#ifdef OC_OUTPUT
			cout << "queries left... first query NOT finished, do other things or wait" << endl;
#endif
			/*
				 violates rule: v-queries should only be accessed in the next frame

				// there are visible-queries => issue the first in queue
				if (this->vQueries.size()) {
#ifdef OC_OUTPUT
					cout << "vQueries left: " << this->vQueries.size() << endl;
#endif
					GeomInstance* nextVQuery = this->vQueries.front();
					this->vQueries.pop_front();

					this->issueQuery(nextVQuery);
				}

				MAYBE WE CAN DO SOMETHING OTHER HERE
				*/
			}
		}

		// processing finished queries could have added entries into the instance queue
		if (this->distanceQueue.size()) {
			GeomInstance* instance = this->distanceQueue.front();
			this->distanceQueue.pop_front();

			if (this->camera.cullBB(instance->geom->getBBMin(), instance->geom->getBBMax()) != OUTSIDE) {
#ifdef OC_OUTPUT
				cout << "instances available: " << this->distanceQueue.size() + 1 << ". checking front instance, has distance = " << instance->distance << endl;
#endif

				// this node was invisible in the last frame => queue it in the invisible-queries queue
				if ((instance->visible && instance->lastFrame == this->frame - 1) == false) {
#ifdef OC_OUTPUT
					cout << "front instance NOT visible in last frame. issue query for visibility" << endl;
#endif

					this->issuePreviouslyInvisibleNode(instance);

				// this node was visible in the last frame => if its a leaf-node quue it in visibile-queries queue
				} else {
#ifdef OC_OUTPUT
					cout << "front instance YES visible in last frame. will be traversed" << endl;
#endif
					instance->lastFrame = this->frame;

					// no children => its a leaf node issue visible-query
					if (instance->children.size() == 0 /* && TODO: QUERY REASONABLE */) {
#ifdef OC_OUTPUT
						cout << "front instance is not LEAF => put in vQueries for query for its visiblity after rendering" << endl;
#endif
						this->vQueries.push_back(instance);
					}

					this->traverseInstance(instance);
				}
			} else {
				this->culledInstances++;
			}
		}

		// no more rendering work to do => issue the left multiqueries
		if (this->distanceQueue.size() == 0) {
			if (this->iQueries.size()) {
#ifdef OC_OUTPUT
			cout << endl << "no more instances but iQueries available => issue them in a multi query" << endl;
#endif
				this->issueMultiQueries();
			}
		}
	}

	// if there are still visibility-queries left, issue them
	if (this->vQueries.size()) {
#ifdef OC_OUTPUT
		cout << "some vQueries still available at the end of the frame: " << this->vQueries.size() << " issue them..." << endl;
#endif
		StateManager::getInstance().enableState("OcclusionQueryState");

		list<GeomInstance*>::iterator iter = this->vQueries.begin();
		while (iter != this->vQueries.end()) {
			GeomInstance* item = *iter++;
			this->issueQuery(item);
		}

		StateManager::getInstance().disableState("OcclusionQueryState");

		this->vQueries.clear();
	}
#ifdef OC_OUTPUT
	cout << "one frame over. frame index= " << this->frame << endl << endl << endl;

	if (this->frame == 5)
		exit(1);
#endif

	SDL_GL_SwapBuffers();

	this->frame++;

	return true;
}

void OCRenderer::handleReturnedQuery(QueryItem q)
{
	GLuint visiblePixels = 0;
	glGetQueryObjectuivARB(q.id, GL_QUERY_RESULT_ARB, &visiblePixels);

	glDeleteQueriesARB(1, &q.id);

	// there are enough visible pixels to contribute to the frame
	if (visiblePixels >= VISIBLE_PIXEL_THRESH) {
		// multiquery failed => we need to query all items individually
		if (q.items.size() > 1) {
#ifdef OC_OUTPUT
			cout << "handleReturnedQuery: was multi, PIXELS AVAILABLE => something visible => issuing individual queries... " << endl;
#endif
			this->queryIndividualNodes(q);

		// single query failed
		} else {
#ifdef OC_OUTPUT
			cout << "handleReturnedQuery: was single, PIXELS AVAILABLE => something visible" << endl;
#endif
			GeomInstance* instance = q.items.front();
			// was previously not visible but is now visible => traverse its children and mark this item as visible
			if ((instance->visible && instance->lastFrame == this->frame - 1) == false) {
#ifdef OC_OUTPUT
				cout << "previously invisible, NOW visible" << endl;
#endif
				this->traverseInstance(instance);
			}

			this->pullUpVisibility(instance);
		}

	// not enough visible pixels to contribute to the frame => mark as invisible
	} else {
		GeomInstance* instance = q.items.front();
		instance->visible = false;

		this->occludedInstances++;

#ifdef OC_OUTPUT
		cout << "handleReturnedQuery: instance becomes invisible with distance = " << instance->recalculateDistance() << endl;
#endif
	}

#ifdef OC_OUTPUT
	cout << endl;
#endif
}

void OCRenderer::traverseInstance(GeomInstance* instance)
{
	instance->lastFrame = this->frame;

	// not a leaf-node
	if (instance->children.size() > 0) {
#ifdef OC_OUTPUT
		cout << "traverseInstance of parent-node with distance " << instance->recalculateDistance() << " => pushing " << instance->children.size() << " children into instances " << endl;
#endif

		if (this->camera.cullBB(instance->geom->getBBMin(), instance->geom->getBBMax()) != OUTSIDE) {
			for (unsigned int i = 0; i < instance->children.size(); i++) {
				instance->children[i]->recalculateDistance();

#ifdef OC_OUTPUT
				cout << "pushed children with distance " << instance->children[i]->distance << " lastFrame = "<< instance->children[i]->lastFrame << " visible = " << instance->children[i]->visible << endl;
#endif

				this->distanceQueue.push_back(instance->children[i]);
			}

			this->distanceQueue.sort(Renderer::geomInstanceDistCmp);

			instance->visible = false;

		} else {
			this->culledInstances++;
		}

	// is a leaf-node => render
	} else {
#ifdef OC_OUTPUT
		cout << "traverseInstance of LEAF-node: RENDER!!! distance = " << instance->distance << endl;
#endif
		glLoadMatrixf(instance->transform.data);
		instance->geom->render();

		this->renderedFaces += instance->geom->getFaceCount();
		this->renderedInstances++;
	}

#ifdef OC_OUTPUT
	cout << endl;
#endif
}

bool OCRenderer::firstQueryFinished()
{
	GLint available;
	QueryItem firstQuery = this->queries.front();
	glGetQueryObjectiv(firstQuery.id, GL_QUERY_RESULT_AVAILABLE_ARB, &available);

	return available;
}

void OCRenderer::pullUpVisibility(GeomInstance* instance)
{
#ifdef OC_OUTPUT
	cout << "pullUpVisibility enter" << endl;
#endif

	// pull up visibility: if this subinstance (if its a subinstance...) is visible, also its parent must be visible
	GeomInstance* parent = instance;
	while (parent) {
#ifdef OC_OUTPUT
	cout << "	node was INVISIBLE => becomes VISIBLE" << endl;
#endif
		parent->visible = true;
		parent = parent->parent;
	}

#ifdef OC_OUTPUT
	cout << "pullUpVisibility exit" << endl;
#endif
}

void OCRenderer::queryIndividualNodes(QueryItem q)
{
#ifdef OC_OUTPUT
	cout << "queryIndividualNodes" << endl;
#endif

	StateManager::getInstance().enableState("OcclusionQueryState");

	list<GeomInstance*>::iterator iter = q.items.begin();
	while (iter != q.items.end()) {
		GeomInstance* instance = *iter++;
		this->issueQuery(instance);
	}

	StateManager::getInstance().disableState("OcclusionQueryState");
}

void OCRenderer::issuePreviouslyInvisibleNode(GeomInstance* instance)
{
#ifdef OC_OUTPUT
	cout << "issuePreviouslyInvisibleNode" << endl;
#endif

	this->iQueries.push_back(instance);
	// batchsize of invisible-queries queue hit => issue as a multiquery
	if (this->iQueries.size() >= INVISIBLE_BATCH_SIZE) {
#ifdef OC_OUTPUT
		cout << "INVISIBLE_BATCH_SIZE hit => multiquery" << endl;
#endif
		this->issueMultiQueries();
	}
}

// issues one single query
void OCRenderer::issueQuery(GeomInstance* instance)
{
#ifdef OC_OUTPUT
	cout << "issueQuery..." << endl;
#endif

	StateManager::getInstance().enableState("OcclusionQueryState");

	QueryItem query;
	query.items.push_back(instance);

	glGenQueries(1, &query.id);

	glBeginQuery(GL_SAMPLES_PASSED, query.id);
		glLoadMatrixf(instance->transform.data);
		instance->geom->renderBB();
	glEndQuery(GL_SAMPLES_PASSED);

	this->queries.push_back(query);

	StateManager::getInstance().disableState("OcclusionQueryState");
}

// issues all invisible-queries in one single query => multiquery
void OCRenderer::issueMultiQueries()
{
	if (this->iQueries.size() == 0) {
#ifdef OC_OUTPUT
	cout << "no multiqueries to issue" << endl;
#endif
		return;
	}

#ifdef OC_OUTPUT
	cout << "issueMultiQueries" << endl;
#endif

	// one query for multiple items => multiquery
	QueryItem query;
	glGenQueries(1, &query.id);

	StateManager::getInstance().enableState("OcclusionQueryState");

	glBeginQuery(GL_SAMPLES_PASSED, query.id);
		// all invisible-queries in the queue are issued in a multi-query
		list<GeomInstance*>::iterator iter = this->iQueries.begin();
		while (iter != this->iQueries.end()) {
			GeomInstance* instance = *iter++;
			query.items.push_back(instance);

			glLoadMatrixf(instance->transform.data);
			instance->geom->renderBB();
		}
	glEndQuery(GL_SAMPLES_PASSED);

	StateManager::getInstance().disableState("OcclusionQueryState");

	this->queries.push_back(query);
	this->iQueries.clear();
}
