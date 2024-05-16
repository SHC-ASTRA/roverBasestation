import React, {useState, useEffect} from 'react'
import useGamepads from '../../hooks/useGamepads.tsx';

export const ArmControl = ({
    topicName = '/astra/arm/control',
    armControlEvent = '/astra/arm/control'
    }) => {
    const [gamepads, setGamepads] = useState({});
    const [gamepadIndex, setIndex] = useState(0);
    // The state is regularly updated by the useGamepads hook
    useGamepads(gamepads => setGamepads(gamepads), 10);

    useEffect(() => {
        for (let i in gamepads) {
            if(!gamepads[i]) continue;
            setIndex(Number(i));
        }
        console.log(gamepads[gamepadIndex])
        if(!gamepads[gamepadIndex]) return;

        socket.emit(armControlEvent, 
            gamepads[gamepadIndex].axes[0], // left stick left/right
            gamepads[gamepadIndex].axes[1], // left stick up/down
            gamepads[gamepadIndex].axes[2], // right stick left/right
            gamepads[gamepadIndex].axes[3], // right stick up down
            gamepads[gamepadIndex].buttons[12].pressed, // d pad up
            gamepads[gamepadIndex].buttons[13].pressed, // d pad down
            gamepads[gamepadIndex].buttons[14].pressed, // d pad left
            gamepads[gamepadIndex].buttons[15].pressed, // d pad right
            gamepads[gamepadIndex].buttons[0].pressed, // b button (button down)
            gamepads[gamepadIndex].buttons[1].pressed, // a button (button right)
            gamepads[gamepadIndex].buttons[2].pressed, // y button (button left)
            gamepads[gamepadIndex].buttons[3].pressed, // x button (button up)
            gamepads[gamepadIndex].buttons[4].pressed, // L (left bumper)
            gamepads[gamepadIndex].buttons[5].pressed, // R (right bumper)
            gamepads[gamepadIndex].buttons[6].value, // ZL (left trigger)
            gamepads[gamepadIndex].buttons[7].value, // ZR (right trigger)
            gamepads[gamepadIndex].buttons[8].pressed, // select / minus
            gamepads[gamepadIndex].buttons[9].pressed // start / plus
        );
    }, [gamepads]);

    const setHome = () => setData({});

    const setPositions = () => setData({});

    return (
        <div className="button-wrapper">
            <button className="control-button" onClick={setHome}>
                Homing
            </button>
            <button className="control-button" onClick={setPositions}>
                Stow
            </button>
            <button className="control-button" onClick={setPositions}>
                Zero
            </button>
            <button className="control-button" onClick={setPositions}>
                Extend
            </button>
            <button className="control-button" onClick={setPositions}>
                Max
            </button>
        </div>
    )
}