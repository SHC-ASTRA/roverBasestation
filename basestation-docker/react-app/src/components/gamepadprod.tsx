        ///////////////////////////////
    // BEGIN GAMEPAD HANDLING
    //////////////////////////////
    
    if(currentGamepadIndex == -1) {
        return (
            <span className='info-danger'>
                Gamepad not Connected
            </span>
        )
    }

    console.log("Attempting to render gamepad index");

    return (
        <span>{gamepads[currentGamepadIndex]}</span>
    )

    // Define controller data variable
    let cGamepadState;
    // Update controller data
    // Filter through gamepads until finding the correct gamepad
    cGamepadState = gamepads[currentGamepadIndex];
    

    if(!cGamepadState || !cGamepadState.index) {
        console.log(`Could not find the connected gamepad with index ${currentGamepadIndex}.`);
        // Set a flag to not render the gamepad display
    }

    /* Status Reading */
    // Trigger indexes
    var leftTriggerID = 6;
    var rightTriggerID = 7;
    // Button indexes
    const button_indexes = ["a","b","x","y","left_bumper","left_bumper","right_bumper","left_trigger","right_trigger","view","menu","d_up","d_down","d_left","d_right"];
    const stick_indexes = ["left_hval","left_vval","right_hval","right_vval"];
    const trigger_indexes = ["left", "right"];

    // Only 15 of the available 16 are used
    for (var i=0; i<=15; i++) {
        // Get button status
        var gamepadButton = cGamepadState.buttons[i];
        var isPressed = gamepadButton == 1.0;
        var isTrigger = false;
        // Triggers
        if (typeof(gamepadButton) == "object" && (i == leftTriggerID || i == rightTriggerID)) {
            isPressed = gamepadButton.pressed;
            isTrigger = true;
        }
        // Other buttons
        else if (typeof(gamepadButton) == "object") { 
            isPressed = gamepadButton.pressed;
        }

        // Triggers
        if (isTrigger) {
            let triggerEls = document.getElementsByClassName("trigger-value");
            if (i == leftTriggerID) {
                triggerEls[0].innerHTML = i + ": " + gamepadButton.value;
                triggerEls[0].setAttribute("value", gamepadButton.value);
            }
            else if (i == rightTriggerID) {
                triggerEls[1].innerHTML = i + ": " + gamepadButton.value;
                triggerEls[1].setAttribute("value", gamepadButton.value);
            }
            /* Update trigger value JSON for communication */
            let offsetIndex = i - leftTriggerID;
            gamepadStatus["trigger_vals"][trigger_indexes[offsetIndex]] = gamepadButton.value.toFixed(4);
        }
    }

    let axes = document.getElementsByClassName("controller-axis");
    // axes[0] horizontal left
    // axes[1] vertical left
    // axes[2] horizontal right
    // axes[3] vertical right
    // Right = increase
    // Left = decrease

    for (let i=0; i<cGamepadState.axes.length; i++) {
        let axis = axes[i];
        // Update the innerhtml of the meter
        axis.innerHTML = i + ": " + cGamepadState.axes[i].toFixed(4);
        // Update the bar length
        axis.setAttribute("value", cGamepadState.axes[i]);
        /* Update stick value JSON for communication */
        gamepadStatus["stick_vals"][stick_indexes[i]] = cGamepadState.axes[i].toFixed(4);
    }

    console.log(gamepadStatus);

    ///////////////////////////////
    // END GAMEPAD HANDLING
    ///////////////////////////////
    
    /*
    * Implement behavior to POST the gamepad outside of the 
    *   context of the React app.
    * Possibly making use of the supported XMLHTTPRequest
    */
    const postGamepad = (id) => {
        return;
    }

    // Implement gamepad polling logic UNNECESSARY due to how react-gamepad handles
    /* useEffect(() => {
        // Implement a gamepad poll handler via a setInterval
        const interval = setInterval(() => {
            // If the gamepad is not set
        }, 1000);
        return () => {
            // Clear the interval
            clearInterval(interval);
        }
    }, [currentGamepad]) // The effect is regenerated each time the gamepad prop is updated */

    return (
        <>
            <p>GAMEPAD</p>
            <div>{JSON.stringify(gamepadStatus)}</div>;
        </>
    )

    // Pre-built HTML page from Two Month 23
    return (
        <div className="container">
        <div className="row pt-3">
            <div className="col text-center">
            <div id="controller-status">
                <div className="fs-2">Controller Status:</div>
                <div className="fs-4" id="controller-not-connected">
                Please connect a controller.
                </div>
                <span className="d-none" id="controller-indicators">
                <div className="buttons">
                    {/*
                            When these buttons are fetched they will return
                            in the order they are. Meaning the first button will be first,
                            second is second, etc
                        */}
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    A
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    B
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    X
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    Y
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    LB
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    RB
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    LT
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    RT
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "8ch", pointerEvents: "none" }}
                    >
                    View
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "8ch", pointerEvents: "none" }}
                    >
                    Menu
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    LS
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    RS
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    Up
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    Dn
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    Lt
                    </button>
                    <button
                    type="button"
                    className="controller-buttons btn btn-info text-center"
                    style={{ width: "5ch", pointerEvents: "none" }}
                    >
                    Rt
                    </button>
                </div>
                <div className="row pt-3" id="triggers">
                    <div className="col-3">{/* Placeholder */}</div>
                    <div className="col-3">
                    <label htmlFor="left-trigger">Left Trigger</label>
                    <meter
                        id="left-trigger"
                        className="trigger-value"
                        min={0}
                        max={1}
                        value={0}
                    />
                    </div>
                    <div className="col-3">
                    <label htmlFor="right-trigger">Right Trigger</label>
                    <meter
                        id="right-trigger"
                        className="trigger-value"
                        min={0}
                        max={1}
                        value={0}
                    />
                    </div>
                    <div className="col-3">{/* Placeholder */}</div>
                </div>
                <div className="row pt-3" id="controller-axes">
                    {/*
                                When these meters are fetched they will return
                                in the order they are. Meaning the first meter will be first,
                                second is second, etc
                            */}
                    <div className="col-3">
                    <label htmlFor="axis-1">Left Horizontal</label>
                    <meter
                        id="axis-1"
                        className="controller-axis"
                        min={-1}
                        max={1}
                        value={0}
                    >
                        0
                    </meter>
                    </div>
                    <div className="col-3">
                    <label htmlFor="axis-2">Left Vertical</label>
                    <meter
                        id="axis-2"
                        className="controller-axis"
                        min={-1}
                        max={1}
                        value={0}
                    >
                        1
                    </meter>
                    </div>
                    <div className="col-3">
                    <label htmlFor="axis-1">Right Horizontal</label>
                    <meter
                        id="axis-3"
                        className="controller-axis"
                        min={-1}
                        max={1}
                        value={0}
                    >
                        2
                    </meter>
                    </div>
                    <div className="col-3">
                    <label htmlFor="axis-1">Right Vertical</label>
                    <meter
                        id="axis-4"
                        className="controller-axis"
                        min={-1}
                        max={1}
                        value={0}
                    >
                        3
                    </meter>
                    </div>
                </div>
                </span>
            </div>
            </div>
        </div>
        </div>
    )