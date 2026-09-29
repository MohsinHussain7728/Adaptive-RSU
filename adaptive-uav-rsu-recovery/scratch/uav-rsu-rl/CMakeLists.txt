# ns-3 scratch program with a layered source tree (rl/, env/, experiment/).
# ns-3 delegates to this file because the directory contains a CMakeLists.txt.

set(UAV_RL_LAYER_SOURCES
    rl/rl-agent.cc
    rl/qlearning-agent.cc
    rl/random-agent.cc
    rl/rl-agent-factory.cc
)

set(UAV_ENV_SOURCES
    env/sim-config.cc
    env/scenario.cc
    env/rsu-manager.cc
    env/uav-controller.cc
    env/network-monitor.cc
    env/topology-observer.cc
    env/state-encoder.cc
    env/reward-function.cc
    env/recovery-strategy.cc
    env/recovery-environment.cc
    experiment/results-writer.cc
    experiment/animation-manager.cc
)

build_exec(
  EXECNAME uav-rsu-rl
  EXECNAME_PREFIX scratch_
  SOURCE_FILES uav-rsu-rl.cc ${UAV_ENV_SOURCES} ${UAV_RL_LAYER_SOURCES}
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/uav-rsu-rl/
)

# Pure C++ unit test of the RL layer: links no ns-3 library at all.
add_executable(uav-rl-agent-test tests/test_rl_agents.cpp ${UAV_RL_LAYER_SOURCES})
set_target_properties(uav-rl-agent-test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_OUTPUT_DIRECTORY}/scratch/uav-rsu-rl/
)
