#!/bin/bash
# run_simulation.sh

set -e

echo "Building project"
mkdir -p build
cd build
cmake ..
make
cd ..

echo "Running CartPole MPC simulation"
./build/cartpole_sim

echo "Generating plots"
python3.13 plot_results.py

echo "Done, you can check mpc_results.png"