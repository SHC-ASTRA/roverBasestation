#!/bin/env bash

# To be run from the main repository directory

cd react-app
# Make use of yarn to install all necessary
# packages inside of the dynamic React frontend
yarn install
# Create a statically build production optimized version
# of the react front-end
yarn build
# Return to the main directory
cd ..