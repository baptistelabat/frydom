//
// Created by frongere on 02/12/2019.
//

#include "WaveLoader.h"
#include "../../cmake-build-debug/_deps/mathutils-src/src/MathUtils/Constants.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;


WaveContainer LoadWavesFromJSON(const std::string &wave_file_json) {

  using Vector = std::vector<double>;
  using VectorOfVector = std::vector<Vector>;

  // Reading
  std::ifstream ifs(wave_file_json);
  auto json_obj = json::parse(ifs);

  auto wave_frequencies = json_obj["wave_frequencies_rads"].get<Vector>();
  auto wave_numbers = json_obj["wave_numbers_m"].get<Vector>();
  int nb_frequencies = wave_frequencies.size();

  auto wave_directions = json_obj["wave_directions_rad"].get<Vector>();
  int nbDir = wave_directions.size();

  auto wave_amplitudes = json_obj["wave_amplitudes_m"].get<VectorOfVector>();

  auto wave_phases = json_obj["wave_phases_rad"].get<VectorOfVector>();

  // Building the structure
  WaveContainer waves;
  for (int idir=0; idir < nbDir; idir++) {
    double wave_direction = wave_directions[idir];

    for (int ifreq=0; ifreq < nb_frequencies; ifreq++) {

      double wave_number = wave_numbers[ifreq];
      double wave_amplitude = wave_amplitudes[idir][ifreq];
      double wave_phase = wave_phases[idir][ifreq];

      waves.push_back(WaveComponent(2. * M_PI / wave_number, wave_amplitude, wave_direction, wave_phase));

    }
  }

  return waves;

}
