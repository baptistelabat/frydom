Install the IRRLICHT module {#module_irrlicht_installation}
==========================

[TOC]

Irrlicht is the default visualization system used by Chrono::Engine. It is one of the few dependency that can't be
managed by cmake. You have to install it on your system

    apt-get install libirrlicht-dev

along with several packages:
- build-essential 
- x11-common 
- libxxf86vm-dev 
- libglu1-mesa-dev 
- freeglut3 
- xorg-dev. 

## Features

Here are the main features:

- Irrlicht shapes and textures can be added as _assets_ to Body objects.
- A default lightweight application framework is provided
  that can be used for default light/camera managements.
- The Irrlicht view of the default application supports some 
  mouse and key interaction:
	- mouse left button for camera rotation
	- mouse right button for camera x z motion
	- mouse wheel rotation for camera forward/backward
	- mouse wheel button to pick & drag objects (only if they have a collision shape!)
	- press 'i' to see a setting panel,
	- press arrows to have x z camera motion, press page up & down for y vertical motion
	- press spacebar to stop simulation
	- press 'p' to advance one timestep a time.
	- press 'print screen' to start saving screenshots to disk
	- etc. (see the 'i' panel)
- There are some easy functions to draw grids, lines, springs, plots.
- Contacts can be optionally drawn with vectors in 3D view
- Link coordinate systems can be plotted on the 3D view
- etc.

