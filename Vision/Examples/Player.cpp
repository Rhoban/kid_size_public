#include "Application/Application.hpp"

#include <iostream>

using namespace std;
using namespace Vision;

class Player : public Application::Application {

  std::string getClassName() const override { return "Player"; }

  void step() {
    Application::step();
    // std::cout << "Nb filters : " << pipeline.filters().size() << std::endl;
  }
};

int main(int argc, char **argv) {
  Player p;
  p.configure(argc, argv);
  p.launch();
}
