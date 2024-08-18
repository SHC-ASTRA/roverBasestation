import React, {useState, useEffect} from 'react'
import useGamepads from '../../hooks/useGamepads.tsx';
import VisualGamepad from '../VisualGamepad.tsx';

export const CoreControl = ({
    controllerScale=2/3,
    coreControlEvent='/astra/core/control',
    ...props
}) => {
    const [gamepads, setGamepads] = useState({});
    const [gamepadIndex, setIndex] = useState(0);
    // The state is regularly updated by the useGamepads hook
    useGamepads(gamepads => setGamepads(gamepads), 10);
    
    for (let i in gamepads) {
        if(gamepads[i]) continue;
        setIndex(Number(i));
    }

    useEffect(() => {

        console.log(gamepads[gamepadIndex])
        if(!gamepads[gamepadIndex]) return;

        if (gamepads[gamepadIndex].buttons[7].value > 0.5) { // one handed driving
            socket.emit(coreControlEvent, gamepads[gamepadIndex].axes[3], gamepads[gamepadIndex].axes[3]);
        } else {
            socket.emit(coreControlEvent, gamepads[gamepadIndex].axes[1], gamepads[gamepadIndex].axes[3]);
        }
    }, [gamepads]);

    return (
        <>
            <p>Gamepad Index: {gamepadIndex}<br />
            Left Stick: {gamepads[gamepadIndex] ? (gamepads[gamepadIndex].axes ? (Math.round(gamepads[gamepadIndex].axes[1] * 100) / 100) : "None") : "None"}<br />
            Right Stick: {gamepads[gamepadIndex] ? (gamepads[gamepadIndex].axes ? (Math.round(gamepads[gamepadIndex].axes[3] * 100) / 100) : "None") : "None"}</p>
        </>
    )
}