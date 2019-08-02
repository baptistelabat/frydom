Benchmark on hydrostatics and stability of a box (bench_NonlinearHydrostaticBox.cpp) {#tutorial_bench_HSNL_box}
================================================

This benchmark corresponds to the simulation of a box in a free-decay test, with a varying density. For a range of
density, the initial box stability condition is not fulfilled, leading the box to stabilize at an odd orientation.
Once the density is out of this range, the box comes back to a more common stabilized orientation.

For more information and results, refer to http://theory.frydom.org/source/benchmark/hydrostatics.html


\include bench_NonlinearHydrostaticBox.cpp
