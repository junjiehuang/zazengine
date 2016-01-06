Overall rule is to write as less code by myself. Use as much libraries as possible. Boost has a wide range of libraries which cover stuff i tried to reinvent in a better and much better tested way. Switch over to Boost as central library. It is needed for LUA-bind, provides platform independent abstraction for threading, networking, reflection and lots of other useful stuff.

Finally make it a real component based system:
  * subsystems compile into library and are loaded at runtime. this mechanism can be used to easily extend the engine.

Design a dynamic Game-Object system which allows to code Game-Object basics in C++ and load them dynamically during runtime - little like the subsystem-stuff.

Boost libraries interesting for zazengine together with a short description of idea where and how it could prove to be useful:
  * Any - Safe, generic container for single values of different value types. For carrying around data with the event-system.
  * Asio - very good networking library.
  * Bind - used for luabind.
  * Conversion - Polymorphic and lexical casts. For carrying around data with the event-system.
  * Flyweight - Design pattern to manage large quantities of highly redundant objects. Could become interesting for handling large number of objects
  * Foreach
  * Function - Function object wrappers for deferred calls or callbacks.
  * Function Types - Boost.FunctionTypes provides functionality to classify, decompose and synthesize function, function pointer, function reference and pointer to member types.
  * Functional/Factory - dynamic and static object creation
  * Fusion - Library for working with tuples, including various containers, algorithms, etc.
  * In Place Factory, Typed In Place Factory - Generic in-place construction of contained objects with a variadic argument-list.
  * Lexical Cast - General literal text conversions, such as an int represented a string, or vice-versa, from Kevlin Henney.
  * MPL - metaprogramming
  * Optional - Discriminated-union wrapper for optional values.
  * Parameter - Boost.Parameter Library - Write functions that accept arguments by name.
  * Pointer Container - Containers for storing heap-allocated polymorphic objects to ease OO-programming.
  * Pool - memory pool management for better performance
  * Property Map - Concepts defining interfaces which map key objects to value objects.
  * Property Tree - A tree data structure especially suited to storing configuration data.
  * Random - random number generation
  * Signals & Signals2 - Managed signals & slots callback implementation ( 2 is thread safe )
  * Smart Ptr - Smart pointer class templates.
  * Test - Support for simple program testing, full unit testing, and for program execution monitoring.
  * Thread - Portable C++ multi-threading.
  * Timer - Event timer, progress timer, and progress display classes.
  * Tuple - Ease definition of functions returning multiple values, and more.
  * Type Traits - Templates for fundamental properties of types.
  * Typeof - Typeof operator emulation.
  * Variant - Safe, generic, stack-based discriminated union container.