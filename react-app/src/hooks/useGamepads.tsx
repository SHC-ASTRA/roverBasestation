import { useEffect, useRef } from 'react';

/*
 * Modified version of the MIT Licensed project react-gamepads by whoisyosuke on GitHub
 * https://github.com/whoisryosuke/react-gamepads
 */

interface GamepadRef {
  [key: number]: Gamepad;
}

export default function useGamepads(hookCallback: (data: GamepadRef) => void, millisecondInterval: number) {
  // Reference values that are is stored outside of the component render scope
  const gamepads = useRef<GamepadRef>([]); // Currently connected gamepads
  // const requestRef = useRef<number>(); // Current animation frame request
  const intervalRef = useRef<number>(); // Current interval request

  var haveEvents = 'ongamepadconnected' in window;

  const addGamepad = (gamepad: Gamepad) => {
    gamepads.current = {
      ...gamepads.current,
      [gamepad.index]: gamepad,
    };

    // Send data to external callback (like React state)
    hookCallback(gamepads.current);
  };

  /**
   * Adds game controllers during connection event listener
   * @param {object} e
   */
  const connectGamepadHandler = (e: Event) => {
    addGamepad((e as GamepadEvent).gamepad);
  };

  /**
   * Finds all gamepads and adds them to context
   */
  const scanGamepads = () => {
    // Grab gamepads from browser API
    var detectedGamepads = navigator.getGamepads();

    // Loop through all detected controllers and add if not already in state
    for (var i = 0; i < detectedGamepads.length; i++) {
      const newGamepad = detectedGamepads[i];
      if (newGamepad && newGamepad !== null) addGamepad(newGamepad);
    }
  };

  // Add event listener for gamepad connecting
  useEffect(() => {
    window.addEventListener('gamepadconnected', connectGamepadHandler);

    return window.removeEventListener(
      'gamepadconnected',
      connectGamepadHandler
    );
  });

  useEffect(() => {
    // Create an interval that updates every millisecondInterval (provided by the hook call)
    intervalRef.current = setInterval(() => {
      // If an "gamepad added" event is not currently running and scanning the gamepads
      if (!haveEvents) scanGamepads();
    }, millisecondInterval);

    // Deconstructor
    return () => clearInterval(intervalRef.current);
  });

  return gamepads.current;
}
