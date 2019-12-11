//
// Created by frongere on 02/12/2019.
//

#include <frydom/frydom.h>
#include <MathUtils/Unit.h>


using namespace frydom;

int main(int argc, char *argv[]) {

  double water_depth = 0.;
  assert(water_depth >= 0.);

  double hs = 3;
  double tp = 9;
  double gamma = 3.3;

  int nb_wave_directions = 10;
  double spreading_factor = 10.; // Between 1 and 100. Defines the bandwidth of the directions

  double mean_wave_direction_angle = 0.;

  double min_wave_frequency = 0.5;
  double max_wave_frequency = 2.;
  int nb_wave_frequencies = 20;

  std::string json_file_name = "sea_state.json";

  FrOffshoreSystem system("sea_state_generator");

  auto ocean = system.GetEnvironment()->GetOcean();

  ocean->GetSeabed()->SetBathymetry(water_depth, NED);

  auto wave_field = ocean->GetFreeSurface()->SetAiryIrregularWaveField();
  wave_field->SetJonswapWaveSpectrum(hs, tp, gamma);
  wave_field->SetDirectionalParameters(nb_wave_directions, spreading_factor, COS2S);
  wave_field->SetMeanWaveDirectionAngle(mean_wave_direction_angle, mathutils::DEG, NED, GOTO);
  wave_field->SetWaveFrequencies(min_wave_frequency, max_wave_frequency, nb_wave_frequencies);


  wave_field->WriteToJSON(json_file_name);

  return 0;
}
