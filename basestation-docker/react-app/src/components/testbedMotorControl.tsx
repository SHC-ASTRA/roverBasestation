import React, { useEffect, useState } from 'react';
import useGamepads from '../hooks/useGamepads.tsx';
import VisualGamepad from './VisualGamepad.tsx';

export default function TestbedControl() {
    const [gamepads, setGamepads] = useState({});

    useGamepads(gamepads => setGamepads(gamepads), 250);

    useEffect(() => {
        let gamepad;
        for (let i in gamepads) {
            if(!gamepads[i]) continue;
            gamepad = gamepads[i];
        }
        console.log(gamepad);
        if(!gamepad) return;

        socket.emit('/core/control', gamepad.axes[0], gamepad.axes[1], gamepad.axes[2], gamepad.axes[3]);
    })

    return (
        <VisualGamepad scale={1/2} gamepadIndex={0}/>
    )
}