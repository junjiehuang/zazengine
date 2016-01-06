Development should happen under the motto: „First make it work, then make it look good and then make it work fast“. Don't waste too much time in small details, try to get a correct result as fast as possible, then make it beautiful both in code and visual appearance. Bevore the renderer is not finished, no big optimizations should happen.

**ALWAYS: find a good game-idea**
  * full 3D with beautiful materials and lighting, no iso&2d
  * playing, creating, experiencing, fascinating (PCEF)
  * should behave like a simulation, independent from the player
  * player can change the simulation by invoking actions
  * no destruction & hard working in the game (e.g. shooter )
  * music: ambient, so game must fit into this kind of music

**Finish Renderer**
  * material-system
  * deferred shadow-maping
  * multi-lights

**Overhaul Architecture**
  * split into separate libs - clean separation into header files needed
  * loading of libs/modules at runtime

**Implement Scripting-Mechanism**
  * depends on the kind of game

**Implement Sound**
  * expand usage of fmod
  * sound-events

**Implement AI**
  * use existing open-source ai
  * use existing open-source pathfinding

**Implement Networking**
  * use existing networking lib