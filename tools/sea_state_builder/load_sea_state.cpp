//
// Created by frongere on 02/12/2019.
//


#include <iostream>
#include "WaveLoader.h"


void AddSwell(float waveLength, float waveHeight, float direction, float phase = 0, bool leftHanded = false) {
  std::cout << "Adding swell component : ";
  std::cout << "lambda(" << waveLength << ") m;\t";
  std::cout << "H(" << waveHeight << ") m;\t";
  std::cout << "beta(" << direction << ") rad;\t";
  std::cout << "phase(" << phase << ") rad;\t";
  std::cout << std::endl;
}


int main(int argc, char *argv[]) {

  WaveContainer waves = LoadWavesFromJSON("swell_components.json");

  // Emulating addition of wave components into Triton...
  auto wave_iter = waves.begin();
  for (; wave_iter != waves.end(); wave_iter++) {
    auto wave = *wave_iter;
    AddSwell(
        (float) wave.m_wave_length_m,
        (float) wave.m_wave_amplitude_m * 2.,
        (float) wave.m_wave_direction_rad,
        (float) wave.m_wave_phase_rad
    );
  }


  return 0;
}
