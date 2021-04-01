#!/bin/bash
# Update numpy library
sudo apt-get install libatlas-base-dev -y

# Install python dependencies
python3 -m pip install ./6RUSRobot/requirements.txt
