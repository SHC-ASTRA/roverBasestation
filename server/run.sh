#!/bin/bash
if ! source /opt/ros/humble/setup.bash; then
    source ~/ros2-humble/install/local_setup.bash
fi

cd ../react-app
if yarn build; then
    cd ../server
    if python3 -m flask run --no-reload --host=0.0.0.0; then
        echo "Goodbye!"
    else
        echo "The server did not build correctly."
    fi
else
    echo "The react app did not build properly"
    cd ../server
fi
