import React, { useState } from 'react';
import { useGamepads } from 'react-gamepads';

export function SimpleGamepadDisplay() {
    // Create a state that updates the rendered component
    // each time it is updated
    const [gamepads, setGamepads] = useState({});
    // Create a hook that updates the state each time the gamepad
    // objects array is updated
    // https://github.com/whoisryosuke/react-gamepads
    // The hook expects an anonymous function that takes the gamepad
    // object array as an argument
    useGamepads(gamepads => setGamepads(gamepads));

    const gamepadDisplay = Object.keys(gamepads).map(gamepadId => {
        const gamepad = gamepads[gamepadId];

        // Button array indexes for triggers
        const triggerIndexes = [6, 7];

        const triggers = gamepad.buttons.filter((button, index) => triggerIndexes.includes(index)); 

        return (
            <div>
                <h2>{gamepad.id}</h2>
                <p>Buttons</p>
                {gamepad.buttons &&
                    gamepad.buttons.map((button, index) => (
                        <div>
                            {index}: {button.pressed ? 'True' : 'False'}
                        </div>
                    ))
                }
                <p>Triggers</p>
                {triggers &&
                    triggers.map((trigger, index) => (
                        <div>
                            {index}: {trigger.pressed ? 'True' : 'False'} | {trigger.value}
                        </div>
                    ))
                }
                <p>Axes</p>
                {gamepad.axes &&
                    gamepad.axes.map((value, index) => (
                        <div>
                            {index}: {value}
                        </div>
                    ))
                }
            </div>
        );
    });

    return (
        <div className="Gamepads">
            <h1>Gamepads</h1>
            {gamepadDisplay}
        </div>
    );
}