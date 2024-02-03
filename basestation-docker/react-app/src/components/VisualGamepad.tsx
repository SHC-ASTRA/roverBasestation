import React, { useState } from 'react';
import { useGamepads } from 'react-gamepads';
import GamepadSvg from './GamepadSvg.tsx';

/* Displays the GamepadSVG Component */

export default function VisualGamepad({
    scale=1,
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
      {gamepads && gamepads[0] && (
        <>
          <GamepadSvg
            directionUp={gamepads[0].buttons[12].pressed}
            directionDown={gamepads[0].buttons[13].pressed}
            directionLeft={gamepads[0].buttons[14].pressed}
            directionRight={gamepads[0].buttons[15].pressed}
            buttonDown={gamepads[0].buttons[0].pressed}
            buttonRight={gamepads[0].buttons[1].pressed}
            buttonLeft={gamepads[0].buttons[2].pressed}
            buttonUp={gamepads[0].buttons[3].pressed}
            bumperLeft={gamepads[0].buttons[4].pressed}
            bumperRight={gamepads[0].buttons[5].pressed}
            triggerLeft={gamepads[0].buttons[6].value}
            triggerRight={gamepads[0].buttons[7].value}
            select={gamepads[0].buttons[8].pressed}
            start={gamepads[0].buttons[9].pressed}
            analogLeft={
              gamepads[0].axes[0] > 0.1 ||
              gamepads[0].axes[0] < -0.1 ||
              gamepads[0].axes[1] > 0.1 ||
              gamepads[0].axes[1] < -0.1
            }
            analogRight={
              gamepads[0].axes[2] > 0.1 ||
              gamepads[0].axes[2] < -0.1 ||
              gamepads[0].axes[3] > 0.1 ||
              gamepads[0].axes[3] < -0.1
            }
            analogLeftDirection={[
              calcDirectionHorizontal(gamepads[0].axes[0]),
              calcDirectionVertical(gamepads[0].axes[1])
            ]}
            analogRightDirection={[
              calcDirectionHorizontal(gamepads[0].axes[2]),
              calcDirectionVertical(gamepads[0].axes[3])
            ]}
            scale={scale}
          />
        </>
      )}
    </div>
  );
}
