This Page describes the principle design and abstract workings of the core architecture together with the mechanics and rules to allow it to work multithreaded.

At its heart the zazengine is an event-simulation. This allows for a very decoupled and dynamic approach of how the objecs in the engine communicate with each other. With the use of this layer of abstraction multithreading, scripting and networking becomes more easy and the architecture more general.

The event-simulation is split into two phases during each iteration: Event Creation and Event Handling.

Phase I: Event creation
In this phase events are created or raised. This is the phase where the iterative "calculations" for the overal global state are done which may or may not result in events.
An example are physics calculations. In this Phase the physics will calculate the new positions of all physical objects and detect collisions between them. When a collision is detected an event is created.

Phase II: Event handling
In this phase the previously created events are distributed to the receivers and the receivers can handle the events.

The two phases are not

In this mechanism the following Objects are involved:
SubSystem (SS)
GameObject (GO)
SubSystemEntity (SSE)

SubSystems are the functional parts of a game which also depend on real-time processing e.g. graphics renderer, physic engine, audio output, ai, input of keyboard/mouse/joystick, networking. SSs can both receive and create events. In Phase I they calculate the new state of the next frame and can create events in this context (see example bevore). In Phase II they can receive events but SSs are normally not heavily involved in event-reception but are more the source of events.

GameObjects are the representation of Objects in the gameworld which need to be handled individually. GameObjects combine multiple SS functionality e.g. a vehicle has a graphic representation, a physical model, an audio output and maybe ai steering. TODO: about the phases I + II, like a translator, Scripting here, call directly to SSs, not sending events, can be proactive not only reactive with update and involved in Phase I.

SubSystemEntities and GameObjects relate very close to each other because the representations of a SS part of a GameObject are handled through a SSE. They are hold by each GO as a composite. TODO: give example

Rules
GOs never ever call directly to each other through their methods, they communicate by sending events to each other.

SSs never ever call directly to GOs or other SSs through their methods, they communicate decoupled by sending events to them.

GOs hovever call directly to SSs through by using the SSEs instances which are available to the GO through the aggregation.

GOs must not call to SSs during Phase I: event creation because the SS may run in different thread and calculate new state. GOs should hold enough state by themself or through their SSEs to prevent this.

Multithreading

Phase I: Event creation
update of SS and GO can be distributed evenly accross N threads (should match number of cores )

Phase II: Event distribution
If this step is about to be implemented multithreaded then GOs are distributed evenly accross N threads and events are then sent to them, so SS must be thread-safe. SS can receive events too, so this must be handled in a thread-safe way too.
the problem with a multithreaded approach in event distribution is that thread-safe SS can lead to lots of lockings when concurrent access happens and implementing an SS as thread-safe can become very complex. under this circumstances, it is better NOT to multithread this phase because it can lead even to worse performance and much more complex code. when scripting is taken into account - which will happen very much during event distribution because the event-handling is central aspect of scripting - then the multithreading becomes event more comples.
NO MULTITHREADING IN THIS PHASE.

BIG TODO: give an example of one interation with 3 fictional GOs which collide with each other and explain exactly the call paths and event distribution. one go is ai controlled, one network controlled, one user controlled locally.