import React, { useEffect, useState } from 'react';
import useGamepads from '../hooks/useGamepads.tsx';
import VisualGamepad from './VisualGamepad.tsx';

export default function TestbedControl({
    controllerScale=2/3,
    ...props
}) {
    const [gamepads, setGamepads] = useState({});
    const [gamepadIndex, setIndex] = useState(0);

    useGamepads(gamepads => setGamepads(gamepads), 250);

    useEffect(() => {
        for (let i in gamepads) {
            if(!gamepads[i]) continue;
            setIndex(Number(i));
        }
        // console.log(gamepads[gamepadIndex], gamepadIndex);
        if(!gamepads[gamepadIndex]) return;

        socket.emit('/core/control', gamepads[gamepadIndex].axes[1], gamepads[gamepadIndex].axes[3]);
    }, []);

    return (
        <VisualGamepad gamepadIndex={gamepadIndex} scale={controllerScale}/>
    )
}