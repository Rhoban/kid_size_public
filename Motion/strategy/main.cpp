#include <iostream>
#include <tclap/CmdLine.h>
#include <kick_model/kick_model_collection.h>
#include "KickValueIteration.hpp"

using namespace std;

int main(int argc, char* argv[])
{
  TCLAP::CmdLine cmd("RhobanServer", ' ', "0.1");
  TCLAP::ValueArg<double> accuracy("a", "accuracy", "Accuracy", false, 0.2, "accuracy", cmd);
  TCLAP::ValueArg<double> angleAccuracy("d", "angle-accuracy", "Accuracy", false, 5, "angle-accuracy", cmd);
  TCLAP::ValueArg<double> tol("t", "tol", "Tolerance", false, 10, "tolerance", cmd);
  TCLAP::ValueArg<double> grassOffset("o", "grass-offset", "Grass offset (0-> against, 180-> with)", false, 180,
                                      "grass-offset", cmd);
  TCLAP::ValueArg<std::string> load("l", "load", "Load JSON", false, "", "load", cmd);
  TCLAP::ValueArg<std::string> json("j", "json", "path to the kick model collection", false, "KickModelCollection.json",
                                    "json", cmd);
  TCLAP::ValueArg<std::string> csvPath("c", "write_csv", "Output for writing a csv file for strategy", false, "",
                                       "write_csv", cmd);
  // TCLAP::ValueArg<std::string> corridorPath("f", "corridor-path", "Output for the corridor condfiguration", false,
  // "",
  //                                           "corridor-path", cmd);
  TCLAP::SwitchArg gnuplot("g", "gnuplot", "write gnuplot format file", cmd, false);
  TCLAP::SwitchArg writeJson("w", "write_json", "write json result file", cmd, false);
  TCLAP::SwitchArg dump("D", "dump", "dump", cmd, false);
  cmd.parse(argc, argv);

  KickStrategy strategy;
  KickValueIteration kickValueIteration(json.getValue(), accuracy.getValue(), angleAccuracy.getValue(), dump.getValue(),
                                        tol.getValue(), grassOffset.getValue());

  if (load.getValue() != "")
  {
    strategy.fromJson(load.getValue());
    kickValueIteration.loadScores(strategy);
    kickValueIteration.populateStrategy(strategy);
  }
  else
  {
    strategy = kickValueIteration.generate();
  }

  if (gnuplot.getValue())
  {
    strategy.gnuplot();
  }

  if (writeJson.getValue())
  {
    Json::FastWriter writer;
    std::cout << writer.write(strategy.toJson());
  }

  if (csvPath.getValue() != "")
  {
    strategy.writeCSV(csvPath.getValue());
  }
}
