# Changelog

This is the Changelog of the FRyDom framework.
This file should be kept up to date following [these guidelines](https://keepachangelog.com/en/1.0.0/)


## [3.0] - 2020-02-04

### Major changes
Refactoring of FRyDoM entire architecture, lead by the new logging system

### New features
Hydrostatic equilibrium solver, validated on several cases.
 

### Minor changes
- mesh manipulation methods for the nonlinear hydrostatic model
- minor refactoring of the equilibrium frame
- reorganisation of the StepFinalize procedure
- docs updated
 

### bugfix
- FrNode stay fix in the body reference frame, when moving the body COG refrence frame
- integral calculations on the clipped mesh corrected
- unnecessary data removed
- resource path missing for HexagonalArticulatedBuoy
- log FrNonLinearHydrostaticForce
- timezone target fixed
- HDB5Tool updated