import React, { useState } from 'react';
import { useGamepads } from 'react-gamepads';
import GamepadSvg from './GamepadSvg.tsx';

/* Displays the GamepadSVG Component */

export default function VisualGamepad({
    scale=1,
    gamepadIndex=0,
    ...props
}) {
  const [gamepads, setGamepads] = useState({});
  useGamepads((gamepads) => setGamepads(gamepads));

  const calcDirectionVertical = (axe) => {
    // Up
    if (axe < -0.2) {
      return "up";
    }
    // Down
    if (axe > 0.2) {
      return "down";
    }
  };
  const calcDirectionHorizontal = (axe) => {
    // Left
    if (axe < -0.2) {
      return "left";
    }
    // Right
    if (axe > 0.2) {
      return "right";
    }
  };

  return (
    <div
      className="Gamepads"
    >
      {gamepads && gamepads[gamepadIndex] && (
        <>
          <GamepadSvg
            directionUp={gamepads[gamepadIndex].buttons[12].pressed}
            directionDown={gamepads[gamepadIndex].buttons[13].pressed}
            directionLeft={gamepads[gamepadIndex].buttons[14].pressed}
            directionRight={gamepads[gamepadIndex].buttons[15].pressed}
            buttonDown={gamepads[gamepadIndex].buttons[0].pressed}
            buttonRight={gamepads[gamepadIndex].buttons[1].pressed}
            buttonLeft={gamepads[gamepadIndex].buttons[2].pressed}
            buttonUp={gamepads[gamepadIndex].buttons[3].pressed}
            bumperLeft={gamepads[gamepadIndex].buttons[4].pressed}
            bumperRight={gamepads[gamepadIndex].buttons[5].pressed}
            triggerLeft={gamepads[gamepadIndex].buttons[6].value}
            triggerRight={gamepads[gamepadIndex].buttons[7].value}
            select={gamepads[gamepadIndex].buttons[8].pressed}
            start={gamepads[gamepadIndex].buttons[9].pressed}
            analogLeft={
              gamepads[gamepadIndex].axes[0] > 0.1 ||
              gamepads[gamepadIndex].axes[0] < -0.1 ||
              gamepads[gamepadIndex].axes[1] > 0.1 ||
              gamepads[gamepadIndex].axes[1] < -0.1
            }
            analogRight={
              gamepads[gamepadIndex].axes[2] > 0.1 ||
              gamepads[gamepadIndex].axes[2] < -0.1 ||
              gamepads[gamepadIndex].axes[3] > 0.1 ||
              gamepads[gamepadIndex].axes[3] < -0.1
            }
            analogLeftDirection={[
              calcDirectionHorizontal(gamepads[gamepadIndex].axes[0]),
              calcDirectionVertical(gamepads[gamepadIndex].axes[1])
            ]}
            analogRightDirection={[
              calcDirectionHorizontal(gamepads[gamepadIndex].axes[2]),
              calcDirectionVertical(gamepads[gamepadIndex].axes[3])
            ]}
            scale={scale}
          />
        </>
      )}
    </div>
  );
}
