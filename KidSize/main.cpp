#include <signal.h>
#include <string>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <unistd.h>
#include <fenv.h>
#include <tclap/CmdLine.h>

// #undef VISION_COMPONENT

#include <RhAL.hpp>
#include <RhIO.hpp>
#include "scheduler/MoveScheduler.h"
#include "buildinfos.h"

#include <robocup_referee/constants.h>

#ifdef VISION_COMPONENT
#include <Binding/Robocup.hpp>
#include <Binding/LocalisationBinding.hpp>
#endif

using namespace std;

/**
 * Global MoveScheduler pointer
 */
MoveScheduler* moveScheduler = nullptr;
#ifdef VISION_COMPONENT
Vision::Robocup* visionRobocup = nullptr;
Vision::LocalisationBinding* locBinding = nullptr;
#endif

/**
 * Quit signal handler
 */
struct sigaction action;
static void signal_handler(int sig, siginfo_t *siginfo, void *context)
{
   //Avoid warning
   (void) sig;
   (void) siginfo;
   (void) context;
   // Exit
    cout << endl << "Quit Requested. Exiting..." << endl;
    if (moveScheduler != nullptr) {
        moveScheduler->askQuit();
    }
#ifdef VISION_COMPONENT
    if (visionRobocup != nullptr) {
        visionRobocup->closeCamera();
    }
#endif
    return;
}

/**
 * Define signal handler for quit signal
 */
static void signal_attach()
{
    memset(&action, '\0', sizeof(action));
    action.sa_sigaction = &signal_handler;
    action.sa_flags = SA_SIGINFO;

    if (sigaction(SIGINT, &action, NULL) < 0) {
        perror("sigaction");
        exit(1);
    }
}

int main(int argc, char **argv)
{
    // Parameters are not used currently
    TCLAP::CmdLine cmd("RhobanServer", ' ', "0.1");
    TCLAP::ValueArg<int> port("p", "port", "Port", false, RhIO::ServersPortBase, "port", cmd);
    cmd.parse(argc, argv);

    if (RhIO::started()) {
        std::cout << "WARNING: RhIO already started, can't change the port, if you" << std::endl;
        std::cout << "         want to do it, recompile with RHIO_SERVER_AUTOSTART to OFF" << std::endl;
    } else {
        std::cout << "Starting RhIO with port " << port.getValue() << std::endl;
        RhIO::start(port.getValue());
    }


    robocup_referee::Constants::field.loadFile("field.json");

    // Enabling exception, for pedantic debugging
    feenableexcept(FE_DIVBYZERO| FE_INVALID | FE_OVERFLOW);
    try {
        buildinfos_print();

        char buffer[1024];
        gethostname(buffer, 1024);
        RhIO::Root.newChild("server");
        RhIO::Root.newStr("server/hostname")
            ->defaultValue(buffer);
        // Initializing RhIO from config directory
        std::cout << "Loading RhIO config" << std::endl;
        RhIO::Root.load("rhio");
        // Initialize move scheduler
        moveScheduler = new MoveScheduler();
        std::cout << "Move scheduler initilized." << std::endl;
        // Initialize and start Vision
#ifdef VISION_COMPONENT
        std::cout << "Initializing Vision" << std::endl;
        visionRobocup = new Vision::Robocup(moveScheduler);
        std::cout << "Vision initialized." << std::endl;
        std::cout << "Initializing LocalisationBinding" << std::endl;
        locBinding = new Vision::LocalisationBinding(moveScheduler, visionRobocup);
        std::cout << "LocalisationBinding initialized." << std::endl;
#endif
        // Handle quit signal
        signal_attach();
        // Start move scheduler
        moveScheduler->execute();
        // Stop low level thrread and quit
        moveScheduler->askQuit();
        delete moveScheduler;
#ifdef VISION_COMPONENT
        if(visionRobocup !=nullptr)
        {
            visionRobocup->closeCamera();
            // delete visionRobocup;
        }
#endif
        std::cout << "Done." << std::endl;
        // return 0;
    } catch(const std::runtime_error& exc) {
        cout << "ERROR: Server died with runtime exception "
             << endl << exc.what() << endl;
    } catch(const std::logic_error& exc) {
        cout << "ERROR: Server died with logic exception "
             << endl << exc.what() << endl;
    } catch(const std::string& exc) {
        cout << "ERROR: Server died with string exception "
             << endl << exc << endl;
    } 
#ifdef VISION_COMPONENT
    if(visionRobocup!=nullptr)
    {
        visionRobocup->closeCamera();
        // delete visionRobocup;
    }
#endif
    return -1;
}
