//
// Created by frongere on 02/12/2019.
//

#ifndef FRYDOM_EE_WAVELOADER_H
#define FRYDOM_EE_WAVELOADER_H

#include <vector>
#include <fstream>


struct WaveComponent {

  WaveComponent(const double &wave_length_m,
                const double &wave_amplitude_m,
                const double &wave_direction_rad,
                const double &wave_phase_rad) :
      m_wave_length_m(wave_length_m),
      m_wave_amplitude_m(wave_amplitude_m),
      m_wave_direction_rad(wave_direction_rad),
      m_wave_phase_rad(wave_phase_rad) {}

  double m_wave_length_m; // Independent with respect to the water depth
  double m_wave_amplitude_m;  // Multiply by 2 to get the wave height
  double m_wave_direction_rad; // Potential problem concerning the coordinate system...
  double m_wave_phase_rad;  // between 0 and 2*pi

};

class WaveContainer {

 public:
  WaveContainer() = default;

  void push_back(const WaveComponent &wave) {
    m_waves.push_back(wave);
  }

  using WaveContainer_ = std::vector<WaveComponent>;
  using WaveConstIter = WaveContainer_::const_iterator;

  WaveConstIter begin() const {
    return m_waves.cbegin();
  }

  WaveConstIter end() const {
    return m_waves.cend();
  }

 private:
  WaveContainer_ m_waves;
};


WaveContainer LoadWavesFromJSON(const std::string &wave_file_json);


#endif //FRYDOM_EE_WAVELOADER_H
