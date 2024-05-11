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

    useEffect(() => {
        for (let i in gamepads) {
            if(!gamepads[i]) continue;
            setIndex(Number(i));
        }
        console.log(gamepads[gamepadIndex])
        if(!gamepads[gamepadIndex]) return;

        socket.emit(coreControlEvent, gamepads[gamepadIndex].axes[1], gamepads[gamepadIndex].axes[3]);
    }, [gamepads]);

    return (
        <>
            <p>{gamepadIndex}</p>
            {/* <p>{gamepads[gamepadIndex]}</p> */}
            {/* <VisualGamepad gamepadIndex={gamepadIndex} scale={controllerScale}/> */}
        </>
    )
}