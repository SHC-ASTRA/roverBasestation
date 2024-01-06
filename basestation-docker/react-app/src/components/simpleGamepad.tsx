import React, { useEffect, useRef, useState } from 'react';
import useGamepads from '../hooks/useGamepads.tsx';

const buttonNames = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'LT', 'RT', 'View', 'Menu', 'LS', 'RS', 'Up', 'Dn', 'Lt', 'Rt', ''];
const chSize = [5,5,5,5,5,5,5,5,8,8,5,5,5,5,5,5,0];

export default function SimpleGamepadDisplay() {
    // Create a state that updates the rendered component
    // each time it is updated
    const [gamepads, setGamepads] = useState({});
    // Create a hook that updates the state each time the gamepad
    // objects array is updated
    // https://github.com/whoisryosuke/react-gamepads
    // The hook expects an anonymous function that takes the gamepad
    // object array as an argument
    useGamepads(gamepads => setGamepads(gamepads), 1000/10);

    useEffect(() => {
        if(!gamepads[0]) return;
        const gamepad = gamepads[0];
        // Emit to the socket.io websocket
        socket.emit('axes', gamepad.axes[0], gamepad.axes[1], gamepad.axes[2], gamepad.axes[3]);
    }, [gamepads])

    const gamepadDisplay = Object.keys(gamepads).map(gamepadId => {
        const gamepad = gamepads[gamepadId];

        // Button array indexes for triggers
        const triggerIndexes = [6, 7];

        const triggers = gamepad.buttons.filter((button, index) => triggerIndexes.includes(index)); 

        return (
            <>
                <div className="row text-center">
                    <h2>{gamepad.id}</h2>
                </div>
                <div className="row text-center">
                    <p>Buttons</p>
                    {gamepad.buttons &&
                        gamepad.buttons.map((button, index) => (
                            <button type="button" className={`btn ${button.pressed ? "btn-success" : "btn-info"}`} style={{"width": `${chSize[index]}ch`, "pointerEvents": "none"}}>{buttonNames[index]}</button>
                        ))
                    }
                </div>
                <div className="row text-center">
                    <p>Triggers</p>
                    {triggers &&
                        triggers.map((trigger, index) => (
                            <div>
                                {index}: {trigger.pressed ? 'True' : 'False'} | {trigger.value}
                            </div>
                        ))
                    }
                </div>
                <div className="row text-center">
                    <p>Axes</p>
                    {gamepad.axes &&
                        gamepad.axes.map((value, index) => (
                            <div>
                                {index}: {value}
                            </div>
                        ))
                    }
                </div>
            </>
        );
    });

    return (
        <div className="container Gamepads">
            <div className="row text-center">
                <h1>Gamepads</h1>
            </div>
            {gamepadDisplay}
        </div>
    );
}