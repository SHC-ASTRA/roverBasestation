import React, { useEffect, useState } from 'react';
import useGamepads from '../hooks/useGamepads.tsx';
import VisualGamepad from './VisualGamepad.tsx';

export default function TestbedControl() {
    const [gamepads, setGamepads] = useState({});

    useGamepads(gamepads => setGamepads(gamepads), 250);

    useEffect(() => {
        if(!gamepads[0]) return;
        const gamepad = gamepads[0];

        socket.emit('/core/control', gamepad.axes[0], gamepad.axes[1], gamepad.axes[2], gamepad.axes[3]);
    })

    return (
        <VisualGamepad scale={1/2}/>
    )
}