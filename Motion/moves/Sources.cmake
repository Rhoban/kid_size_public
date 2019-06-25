set (SOURCES
    ApproachPotential.cpp
    ApproachMove.cpp
    AutonomousPlaying.cpp
    Penalty.cpp
    StandUp.cpp
    Move.cpp
    Moves.cpp
    Search.cpp
    STM.cpp
    Replayer.cpp
    Head.cpp
    Robocup.cpp
    Playing.cpp
    IMUTest.cpp
    Walk.cpp
    GoalKeeper.cpp
    Placer.cpp
    Kick.cpp
    Arms.cpp

    VisionLogMachine.cpp
    # LogMachine.cpp
    
    GoalKick.cpp
    ReactiveKicker.cpp
    TCHighKick.cpp
)

if (csa_mdp_experiments_FOUND)
  set(SOURCES
    ${SOURCES}
    KickController.cpp
#    LearnedApproach.cpp
#    MDPKickController.cpp
    # QKickController.cpp
    MCKickController.cpp
    ClearingKickController.cpp
    PenaltyKickController.cpp
    )
endif(csa_mdp_experiments_FOUND)


set_source_files_properties(Placer.cpp PROPERTIES COMPILE_FLAGS "-g -O0")
