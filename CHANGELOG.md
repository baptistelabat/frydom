# Changelog

This is the Changelog of the FRyDom framework.
This file should be kept up to date following [these guidelines](https://keepachangelog.com/en/1.0.0/)

## [3.1] - 2020-02-06

### Changed
- minor changes in the hydrostatic equilibrium solver
- minor changes in the equilibrium frame

## [3.0] - 2020-02-04

### Major changes
Refactoring of FRyDoM entire architecture, lead by the new logging system

### Added
Hydrostatic equilibrium solver, validated on several cases.
 

### Changed
- mesh manipulation methods for the nonlinear hydrostatic model
- minor refactoring of the equilibrium frame
- reorganisation of the StepFinalize procedure
- docs updated
 

### Fixed
- FrNode stay fix in the body reference frame, when moving the body COG reference frame
- integral calculations on the clipped mesh corrected
- unnecessary data removed
- resource path missing for HexagonalArticulatedBuoy
- log FrNonLinearHydrostaticForce
- timezone target fixed
- HDB5Tool updated

## [2.1] - 2019-08-06

### Added
- New constraint type have been added
- New demos

### Changed
- New architecture for hydrodynamic interactions, validation and complex test cases
- Refactoring of Nonlinear hydrostatics and Froude-Krylov
- Refactoring of HDB5Tool python utility
- Update of the documentation with nice images (thanks Caroline Leguludec ;-) 
- Benchmark data have been moved to Amazon S3 cloud storage

### Fixed
- Unnecessary data removed
- Log FrNonlinearHydrostaticForce
- Resource path for demo_HexagonalArticulatedBuoy
- Timezone target fixed