This Page describes the principle design and abstract workings of the core functionality: the event-simulation together with the mechanics and rules to allow it to work multithreaded.

At its heart the zazengine is an event-simulation. This means

Phase I: Event creation
Phase II: Event distribution/consumption

Multithreading

Rules
GameObjects (GO) never ever call directly to each other through their methods, they communicate by sending events to each other.

SubSystems (SS) never ever call directly to GOs or other SSs through their methods, they communicate decoupled by sending events to them.

GOs hovever call directly to SSs through by using the SubSystemEntity (SSE) instances which are available to the GO through the aggregation.

Phase I: Event creation
update of SS and GO can be distributed evenly accross N threads (should match number of cores )

Phase II: Event distribution
If this step is about to be implemented multithreaded then GOs are distributed evenly accross N threads and events are then sent to them, so SS must be thread-safe. SS can receive events too, so this must be handled in a thread-safe way too.
the problem with a multithreaded approach in event distribution is that thread-safe SS can lead to lots of lockings when concurrent access happens and implementing an SS as thread-safe can become very complex. under this circumstances, it is better NOT to multithread this phase because it can lead even to worse performance and much more complex code. when scripting is taken into account - which will happen very much during event distribution because the event-handling is central aspect of scripting - then the multithreading becomes event more comples.
NO MULTITHREADING IN THIS PHASE.