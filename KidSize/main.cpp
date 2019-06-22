#include <signal.h>
#include <string>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <unistd.h>
#include <fenv.h>
#include <tclap/CmdLine.h>
#include "backtrace.h"

#include <RhAL.hpp>
#include <RhIO.hpp>
#include "scheduler/MoveScheduler.h"
#include "buildinfos.h"

#include <robocup_referee/constants.h>

#include <Binding/Robocup.hpp>
#include <Binding/LocalisationBinding.hpp>
#include "rhoban_utils/logging/logger.h"

rhoban_utils::Logger logger("main");

using namespace std;

/**
 * Global MoveScheduler pointer
 */
MoveScheduler* moveScheduler = nullptr;
Vision::Robocup* visionRobocup = nullptr;
Vision::LocalisationBinding* locBinding = nullptr;

/**
 * Quit signal handler
 */
struct sigaction action;
static void signal_handler(int sig, siginfo_t* siginfo, void* context)
{
  // Avoid warning
  (void)sig;
  (void)siginfo;
  (void)context;
  // Exit
  logger.error("Quit requested, exiting");
  if (moveScheduler != nullptr)
  {
    moveScheduler->askQuit();
  }

  if (visionRobocup != nullptr)
  {
    visionRobocup->closeCamera();
  }

  return;
}

static void signal_abort(int sig)
{
  fclose(stdout);
  if (sig == SIGFPE)
  {
    logger.error("[!] Received SIGFPE signal, backtrace:");
  }
  else
  {
    logger.error("[!] Received ABORTING signal, backtrace:");
  }
  // get void*'s for all entries on the stack
  std::cerr << backtrace() << std::endl;

  exit(1);
}

/**
 * Define signal handler for quit signal
 */
static void signal_attach()
{
  memset(&action, '\0', sizeof(action));
  action.sa_sigaction = &signal_handler;
  action.sa_flags = SA_SIGINFO;

  if (sigaction(SIGINT, &action, NULL) < 0)
  {
    perror("sigaction");
    exit(1);
  }
}

int main(int argc, char** argv)
{
  // Parameters are not used currently
  TCLAP::CmdLine cmd("RhobanServer", ' ', "0.1");
  TCLAP::ValueArg<int> port("p", "port", "Port", false, RhIO::ServersPortBase, "port", cmd);
  TCLAP::SwitchArg noVision("n", "no-vision", "No vision", cmd, false);
  cmd.parse(argc, argv);

  bool hasVision = !noVision.getValue();

  if (RhIO::started())
  {
    logger.warning("RhIO already started, can't change the port, if you want to do it, recompile with "
                   "RHIO_SERVER_AUTOSTART to OFF");
  }
  else
  {
    logger.log("Starting RhIO with port %d", port.getValue());
    RhIO::start(port.getValue());
  }

  robocup_referee::Constants::field.loadFile("field.json");

  // Enabling exception, for pedantic debugging
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
  signal(SIGABRT, &signal_abort);
  signal(SIGFPE, &signal_abort);

  buildinfos_print();

  char buffer[1024];
  gethostname(buffer, 1024);
  RhIO::Root.newChild("server");
  RhIO::Root.newStr("server/hostname")->defaultValue(buffer);
  // Initializing RhIO from config directory
  logger.log("Loading RhIO config");
  RhIO::Root.load("rhio");

  // Initialize move scheduler
  moveScheduler = new MoveScheduler();
  logger.log("Move scheduler initialized");
  // Initialize and start Vision
  if (hasVision)
  {
    logger.log("Initializing Vision");
    visionRobocup = new Vision::Robocup(moveScheduler);
    logger.log("Vision initialized.");
    logger.log("Initializing LocalisationBinding");
    locBinding = new Vision::LocalisationBinding(moveScheduler, visionRobocup);
    logger.log("LocalisationBinding initialized.");
  }
  // Handle quit signal
  signal_attach();
  // Start move scheduler
  moveScheduler->execute();
  // Stop low level thrread and quit
  moveScheduler->askQuit();
  delete moveScheduler;
  if (hasVision)
  {
    if (visionRobocup != nullptr)
    {
      visionRobocup->closeCamera();
      // delete visionRobocup;
    }
  }
  logger.log("Done.");

  if (visionRobocup != nullptr)
  {
    visionRobocup->closeCamera();
    // delete visionRobocup;
  }

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
