!!! TOTALLY OUTDATED !!!
!!! THIS PAGE EXISTS FOR HISTORICAL REASONS !!!

Architectural Design-Issues and Goals for the Zazengine

The overal goal of this project is to develop an architecture for “parallel discrete realtime agent-based event simulation”. This kind of architecture allows the development of games, crowd-simulations,... and is completely different from standard GUI-based apps. These are user-input driven while these simulation apps run completely without user-inputs.

Event-simulation
The heart of the app is a powerful Event-system which allows the raising and distribution of events to different targets which react in different manner to events and raise new events.

Agent-based
The agents are the entities which raise events and react to events. Agents can for example be AI-controlled Entities. A User playing a game is just a special case.

Parallel
The architecture makes it possible to utilize multi-core machines by distributing workload between threads.

Discrete
The simulation is continuously updated in discrete time-step. The time needed for each update depends on the complexity of the simulation.

Realtime
The time needed for each update should so that the simulation can be updated 30-60 times a second.

Agent Object-Model
The Agent Object-Model is the heart of the architecture which ties everything together. Every Agent is defined through xml which defines its structure like properties and subsystem-parts and through scripts which defines its behaviour (which is in fact the handling and raising of events). It is also possible to specify a already implemented Agent-Class in C++ which will be instantiated to represent this agent. Within this Class some stuff could be already implemented to prevent cumbersome or impossible to implement in script stuff.
Every Agent can be part of multiple different subsystems (e.g. has a graphical representation through the grraphicsubsystem, a physical calculation through physicssubsystem and a audiooutput through audiosubsystem). These parts are accumulated in a subsystemsentity-composite.
Each Agent has an arbitrary amount of arbitrary properties (e.g. health, ammo,...), which are implemented through a powerful and flexible property-system. Properties can be inherited from other Agents, properties can also be added and remove during runtime. Each Agent can receive events and react on them. This is normally done through scripting: the EventSystem distributes an event to this agent and the event is delegated to script if scripthandlers are present.

Modularisation
- SubSystem, SubSystemEntity, Controler, parallel (data parallel)
The simulation is split into multiple subsystems like phyics, graphics, audio, animation,... SubSystems expose the instantiation of SubSystemEntities which are objects represnting data and functionality for this subsystem e.g. a graphicsubsystementity would hold descriptions about its appearance (model, transformation). Subsystems also must exchange data. This is done through the SubSystemEntities-Controler structure:
- each subsystementity can have controlers registered with it. these controlers allow for interchange of data between subsystementities through a generic interface. e.g. graphicsentity can have controlers which can modify: vertices, vertexindices, vertextnormals, ... most important: transformation for positional and rotational changes. an audioentity can have controler which can modify: position, velocity, the distance how far the sound can be heard...
- the adding/removing of controlers occurs in the ISubSystemEntity-Interface through add/removeControler( IControler**);
- the controler hast just one pure virtual method which allowes to query the type of the controler.
- for each controlertype a new interface should be introduced e.g. the renderer provides controler-interfaces like ITransformationControler. the AudioSystem provides IAudioSourceControler.
- during each synced step of the main-loop the subsystementity casts each controler based upon its exposed type to its interface and calls update on it and passes itself as argument.**

It should be easy to exchange same types of subsystems e.g. when a game has physics support to make it easy to change to a different physicslibrary.

Runtime Object-Model
The runtime Object-Model is encapsulated within a abstract Class Object from which every Class inherits which must support one or more of the following features: RTTI, Streaming or Networking.
RTTI implements runtime-typeinformation which is sometimes necessary to overcome the generic interfaces when specific types are needed.
Streaming allows for dynamically loading of new objects into the simulation.
Networking allows the distribution of the simulation over network e.g. some objects or subsystems run on different machines.

Event-System
- Event-types, Dispatching

Core-System
- loading of SubSystems (libraries)

The main-loop does just four things during each iteration: dispatch the events accumulated from the previous iteration, call prepareProcess, process and finalizeProcess on each subsystem. Because the subsystems can work parallel in a thread three calls are necessary. It is not possible without an extreme synchronization overhead to let subsystem run all the time parallel in a thread and do synchronisation just when subsystem-external data or calls are done. Because of this it was decided that parallel processing just occurs in between the two calls process and finalizeProcess. prepareProcess is used to transfer data between subsystems. When process is called upon a async SubSystem it will release a semaphore upon the parallel worker thread is blocking and returns immediately without serious delay to the main-loop, processing will then occur within the thread. When process is called upon a synched subsystem it will to its work and block the main-loop until it has finished. After all subsystems got its process called, each subsystem got its finalizeProcess called. A async subsystem will tell its thread to stop looping (but not executing) and grab the semaphore and as soon the thread has went out of its inner loop and released the semaphore (upon the main-thread blocks) the main-thread will continue. A synched subsystem will probably do nothing during the call to finalizeProcess. As you can see, it is important to place async subsystems in the list bevore synched because they can start running while the synched subsystems are blocking the main-loop.
It is imporant that no call to other subsystem or update of subsystem-external data must occur within these two calls otherwise one would run into sync-problems except when there are no async subsystems.

Scripting
- binding of core- and subsystem-methods, event-handling, event-raising

Platform-Independent
- Abstraction/encapsulation of system-stuff like semaphores, threads, fileaccess,...


Proof-Of-Concept Project
Use this architecture to build a game/simulation with the following content:

As subsystems as much opensource or free available libraries are used, so the subsystems act just as a bridge to the library.

graphics: ogre3d
animations: ogre3d
audio: fmod
input: SDL
AI: http://www.ai4g.com/ http://ai4games.sourceforge.net/
physics: ode/bullet

- sound output with fmod
- physicsimulation with ODE (can move instances from the scene)
- designed with dynamic modularity - each subsystem is a lib dynamically loaded at runtime
- scripting with lua - handle and raise events.
- keyboard & mouse input
- parallel subsystems

Renderer:

implement occlusion queries (chc++?):
we have the depth-buffer from the geometry-stage upon which we can perform the occlusion queries.

straight-forward approach: at the end of the frame issue one single batched occlusion query for all previously invisible objects and one single for each visible. at the beginning of the next frame, check if the queries have finished. if the batched single query returns a value ( pixel that passed the depth test ) > TRESHHOLD, then the invisible objects need to be queried individually. if a query for a previously visible object fails, then it shall be occluded in this frame.

we are behind one frame.

Multithreaded approach:
because the engine should be multithreaded, the complete data-pipeline and subsystem-intervweaving and communication must be designed with the multithreading in mind.

- no concurrent data-access (e.g. matrices, positions, vectors,...) should occur between subsystems (except event-queuing) to prevent slowing down because of sync-locks.
- no shared R/W data possible (e.g. orientation matrices), only shared R data possible (e.g. meshes)
- each subsystem entity stores R/W data locally and updates it just at each sync-point which occurs at the end of each frame
- update occurs hardcoded for now e.g. graphics pulls orientation matrix from physics

event-system:
- all changes through events. e.g. movedTo(x,y,z)...
- two ways of processing events: A) synchronically during sync-point at end of frame B) asynchronically when each subsystem runs
- A pro: no synchronization issues, contra: event-processing can use up quite some time
- B pro: events processed within async subsystem process better utilizes parallelism, contra: sync when generating other events
- example:
user presses forward-button. user-ai reads the buffered input and generates an moveForward-Event and queues it into the event-queue. during the next sync-point the queue is iterated and the events dispatched to all listeners which registered to the event. the user-character GameObject (UC-GO) has subscribed for it and receives the event. upon receiving the event in the GO, a function in the script of the GO is called to handle the event. the physics-subsystementity has subscribed to this event and will receive it. when the sync-point has ended the subsystems start to run and will process their queued events. the physic subsystem will iterate through all entities and processes their events. when it comes to the moveforward-event of the UC-GO it will calculate the new position and updates the orientation-matrix. then the next sync-point comes and the renderer grabs the new orientation-matrix from the physics which will then show up as the new position when rendered.

event-system:
- different approach: ai, audio and graphics subsystementity receive moveForward too and update their positions too when processing the event => no need for pulling data in sync-points. problem: physics needs to do physics on it first => physics generates setOrientation-event in the next step. problem: 2-3 frames deferred. during 1st frame: keypressed, between 1st and 2nd frame: dispatching of event to physics. 2nd frame: physics updates orientation and generates setOrientation-event. between 2nd and 3rd frame: dispatching of event to ai, audio and graphics subsystemsentity. 3rd frame: changes show up/ai reacts to it.
- question: how to integrate it with scripting?

subsystems generate events themselve:
e.g. physics-system can generate a stream of setOrientation when an object falls down from somewhere or is moved arround by laws of physics. or collisions....

events will also be used to communicate through net

following questions need to be solved:
1) define a working event-registration and event-callback mechanism.
2) how do subsystems send events (e.g. physics sends setOrientation once per frame, or collidesWith upon collision )?
3) how does an input (e.g. keypressed) reaches a gameobject or anything in the engine in a clean, decoupled and generic way?
4) how integrate event-system with lua?

ad 1)
the problem is, that events can occur in the context of a gameobject and/or globally. a global event is e.g. keypressed, a 'local' event is e.g. setPosition or collidesWith. so it should be possible to distribute events in two ways: globally or locally. a global event is dispatched to all listeners subscribed to this event, the local-event is dispatched to the according gameobject regardless it has subscribed for it or not. it is then in the listeners interest to react on it accordingly. so there is a need to add a target to the event to mark it as "locally". when no target is specified the event is considered as global.

ad 2) the subsystem uses the global event queue to send events: it creates a new event and enqueues it in the queue. the queue is processed at the next sync-point where all events will be dispatched to their listeners (globally or locally). of course one can argue that the subsystem can also shortcut and directly sends local events to the gameobject with go->sendEvent(...). the problem is, that the gameengine is multithreaded, so subsystems may run asynchronically. these subsystems MUST NOT send events directly to the go because of concurrent modifications possible through other subsystems. only subsystems guaranteed to run in the main-thread can use this short-cut.
global events can only be distributed through the global event queue.

ad 3)

ad 4)


Engine Startup/Shutdown
heavy use of singleton-pattern
subsystems accessible through e.g. a root-singleton

Memory-Management
write own allocator to reduce dynamic allocation during mainloop as much as possible. The key is to allocate in the beginning a single block as big as possible (e.g. 512 MB) and then manage it with special paradigms like stack, ...
take care of memory layout and packing
take care to implement Cache-Friendly

Math
use a special math-library for games which provides math for vectors, matrices, quaternion,...

C++ Stuff
Collections. Take care of memory allocation patterns. the more general purpose the more caution. e.g. STL makes heavy use of dynamic allocation. either write own collection library or use libs specially designed for low memory consumption and/or for use in embedded systems e.g. STLPort
Exceptions provide elegant way to catch errors but could cause very expensive stack-rollbacks
UID Beware of string identifiers because of high cost of comparisons of two strings within a tight loop compared to the comparison of two ints. explore the use of hashcodes.
because of special allocators (and the implicit defragmentation) empower smart-pointers or handles instead of plain old direct pointers.

File Management
encapsulate platform dependent filesystem access on platform-independent api for platform independency.
asynchronus data-loading from file can be implemented behind such an API. This empowers the engine to do streaming.
organize files as contiguous bulks e.g. zip file to minimize seek overhead

Resource Management
loading and setup of resources (textures, sound, meshes, materials,...)
lifecycle of a resource very important: when exactly need to be in ram/vram and when about to be deleted

Main-Loop
multicore

Object-Model
- inter-gameobject-communication by event/message system (implement a powerful event-system which can also be used and extenden by scripting)
- property system
- state caching for coherency because of intermediate results and multithreading results
- multicore: the subsystem posts the result as an event in the game object and/or alters the state caceh (the posting is in fact an alteration of the state-cache)
- composite pattern for aggregates and dependencies (vehicles, platforms)
- game objects are at each time in one or more well defined state(s) and can switch into multiple states
- a state defines behaviour on events and behaviour ouver time (e.g. „take cover“ will react differently on the event „shot“ then „attack“ , also the behaviour over time is in this case different then attack – the object will try to find cover for some time instead of shooting)
- events are either system/subsystem or script generated (e.g. subsystem event: „collides with“, „in line of sith with“,...)
- events can trigger other events
- events can call system/subsystem functionality (e.g. play/stop animation, play/stop sound, play or activate/deactivate effect,..)
- most calls to system/subsystems are internally wraped as tasks and exectuted when the subsystem is updating or the tasks are executed in an execution thread.

Script System
- must be integrated seamlessley with the game object system
- define new events
- react on any event
- call system/subsystem functionality
- should be attachable very easy to each game object
- express global gameflow/gameobjectives by vector-statemachine (multiple states active at one time)
powerful but efficient (python, squirrel,?)

Networking
use existing lib

AI
find existing lib
tight integration with object-states

Sound
use OpenAL + alut: 3D-sound, doppler effect ?

Input
use SDL because OIS not working

Further Readings:
Modern C++
Game Gems 1
Game Gems 2
Mathematics for computergraphics [28](28.md)
Advanced global illumination [8](8.md)
Large scale c++ software design [27](27.md)
9, 10?

Notes done in Mail:

SubSystem Entities
gameobject is almost always part of multiple subsystems e.g. renderer, physics and audio .each subsystem requires specific data, functionality and definitions from the gameobject to know how to represent it. e.g. the renderer needs a model, materialdefinitions, textures for visual representation, orientation in space,...; physics need boundary descriptions either in vertices or in functional form (sphere equation,...) mass parameters, orientation in space; audio needs stuff like velocity, position of the audiosource,... this stuff must not be handled through the propertysystem because its system inherent. the propertysystem ist just there to allow scripters and data-definitions to define as many as necessary properties for a specific gameobject e.g. health, mana, ammo,... the subsystem specific stuff is handled by introducing a factory method (in fact: an abstract factory method -> the subsystem becomes an abstract factory) which instantiates a concrete implementation of a so called SubSystemEntity. This entity encapsulates all data and functionality a subsystem needs to have to operate on a gameobject. A big thing is that SubSystemEntities of a GameObject must be able to communicate with each other without knowing exactly who they are. e.g. the physics subsystems regularly updates the gameobjects orientation matrix. sometimes (as fast as rendering can happen) the renderer will get the orientationmatrix from his subsystementity of the according gameobject so either the physicssubsystementity must push the data to the renderersubsystementity or the rendererssentity pulls it from the physicsssentity (the last one would be the best choice). Initialization happens then as follows:the core iterates over all dataelements of a gameobject xml-definition. when it detects a section for a subsystem (e.g. 

&lt;renderer&gt;

) it searches the according subsystem and query for a subsystem entity. then the dataelement for this subsystementity is passed to the entity which knows how to initialize it self.: XMLElement subSystemDefinition = XXX;string subSystemType = subSystemDefinition.getName();SubSystem**subSys = Core::getSubSystemByType(subSystemType );if (subSys != 0 )  SubSystemEntity** ent = subSys->createEntity();   ent->initialize(subSystemDefinition);endif

subsystem layering
all spacial movement is done through physics => each object which wants to be moved must have a physicentityinstance. animation is not considered as spacial movment.ai controls objecmovment on a highlevel: moveTo, accelerate, turnLeft,... these calls do regular calls to physics to update positions and the ai itself controls if the movement has finished.

http://www.codeproject.com/kb/cpp/allocator.aspx

Scene: holds the scenegraph, allows adding and manipulation of nodesSceneGraph: hierarchical representation of the scene in nodeseSceneGraphnode: a node withing the scenegraph holding information for the renderer ( geominstance, effects/material, light,...)Renderer: traverses the scenegraph, sorting (transparent,...), visibility culling ( frustum culling, occlusion culling), multiple passes for effects/materials GeomTypes: represents different types of renderable objects (mesh, sphere, teapot, volumes, sprites, text, particles)GeomInstance: an instance of a geomtypeEffects/Materials: holds textures, shaders and code when multiple passes are necessary

boost-containers with memorypool-allocators

- each subsystementity can have controlers registered with it. these controlers allow for interchange of data between subsystementities through a generic interface. e.g. graphicsentity can have controlers which can modify: vertices, vertexindices, vertextnormals, ... most important: transformation for positional and rotational changes. an audioentity can have controler which can modify: position, velocity, the distance how far the sound can be heard... - the adding/removing of controlers occurs in the ISubSystemEntity-Interface through add/removeControler( IControler**);- the controler hast just one pure virtual method which allowes to query the type of the controler. - for each controlertype a new interface should be introduced e.g. the renderer provides controler-interfaces like ITransformationControler. the AudioSystem provides IAudioSourceControler.- during each synced step of the main-loop the subsystementity casts each controler based upon its exposed type to its interface and calls update on it and passes itself as argument.**

Artificial Intelligence for Games
http://www.ai4g.com/http://ai4games.sourceforge.net/

Stroustrup: C++ Scott Meyers: Effective STLAndrei Alexandrescu: Modern C++ DesignAllocatoren