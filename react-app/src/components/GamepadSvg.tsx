import React from "react";

function GamepadSvg({
    buttonDown,
    buttonRight,
    buttonLeft,
    buttonUp,
    directionUp,
    directionDown,
    directionLeft,
    directionRight,
    bumperLeft,
    bumperRight,
    triggerLeft,
    triggerRight,
    analogLeft,
    analogRight,
    analogLeftDirection,
    analogRightDirection,
    select,
    start,
    activeColor = "#2F80ED",
    inactiveColor = "#E0E0E0",
    scale,
    ...props
}) {

    const createTransform = (direction) => {
        switch (direction) {
            case "up":
                return "translateY(-10px)";
            case "down":
                return "translateY(10px)";
            case "left":
                return "translateX(-10px)";
            case "right":
                return "translateX(10px)";

            default:
                return "";
        }
    };

    // Return the SVG
    return (
        <svg
            id="Layer_1"
            data-name="Layer 1"
            xmlns="http://www.w3.org/2000/svg"
            version="1.1"
            xmlnsXlink="http://www.w3.org/1999/xlink"
            viewBox="0 0 580 434"
            fill="none" {...props}
        >
            
            {/* Controller body */}

            <path
                className="backgrounds"
                d="M505.9,101c-16.3-10.4-4.5-16.3-21.4-29.2-16.8-12.9-85.1-34.6-97-24.7s-25.2,11.9-25.2,11.9h-144.4s-13.4-2-25.2-11.9c-11.9-9.9-80.2,11.9-97,24.7-16.8,12.9-5.1,18.8-21.4,29.2-16.3,10.4-58.8,153.9-58.8,153.9,0,0-55.4,159.8,43.5,179.1,0,0,24.2-15.3,45-40.1,7.8-9.3,46.4-70.9,66.9-70.9h234c18.5,0,66,64.9,71.1,70.9,20.8,24.7,45,40.1,45,40.1,99-19.3,43.5-179.1,43.5-179.1,0,0-42.5-143.5-58.8-153.9h.2ZM438.1,98.3c13.7,0,24.9,11.2,24.9,24.9s-11.2,24.9-24.9,24.9-24.9-11.2-24.9-24.9,11.2-24.9,24.9-24.9ZM400,136.4c13.7,0,24.9,11.2,24.9,24.9s-11.2,24.9-24.9,24.9-24.9-11.2-24.9-24.9,11.2-24.9,24.9-24.9ZM332.2,145.4c8.8,0,15.9,7.1,15.9,15.9s-7.1,15.9-15.9,15.9-15.9-7.1-15.9-15.9,7.1-15.9,15.9-15.9ZM142.2,209.4c-27.1,0-49.1-22-49.1-49.1s22-49.1,49.1-49.1,49.1,22,49.1,49.1-22,49.1-49.1,49.1h0ZM256.5,266.8c0,1.7-1.4,3.1-3.1,3.1h-22.4v22.5c0,1.7-1.4,3.1-3.1,3.1h-24.2c-1.7,0-3.1-1.4-3.1-3.1v-22.5h-22.4c-1.7,0-3.1-1.4-3.1-3.1v-24.2c0-1.7,1.4-3.1,3.1-3.1h22.4v-22.5c0-1.7,1.4-3.1,3.1-3.1h24.2c1.7,0,3.1,1.4,3.1,3.1v22.5h22.4c1.7,0,3.1,1.4,3.1,3.1v24.2ZM249.1,177.2c-8.8,0-15.9-7.1-15.9-15.9s7.1-15.9,15.9-15.9,15.9,7.1,15.9,15.9-7.1,15.9-15.9,15.9ZM365.4,299c-27.1,0-49.1-22-49.1-49.1s22-49.1,49.1-49.1,49.1,22,49.1,49.1-22,49.1-49.1,49.1ZM438.1,226.3c-13.7,0-24.9-11.2-24.9-24.9s11.2-24.9,24.9-24.9,24.9,11.2,24.9,24.9-11.2,24.9-24.9,24.9ZM479.2,186.2c-13.7,0-24.9-11.2-24.9-24.9s11.2-24.9,24.9-24.9,24.9,11.2,24.9,24.9-11.2,24.9-24.9,24.9Z"
                fill="#C4C4C4"
            />

            {/* CIRCLE BUTTONS */}

            <g>
                {/* button_down */}
                <circle
                    className="button_down"
                    cx="438.1"
                    cy="201.4"
                    r="18.8"
                    fill={buttonDown ? activeColor : inactiveColor}    
                />
                {/* button_right */}
                <circle
                    className="button_right"
                    cx="479.2"
                    cy="161.3"
                    r="18.8"
                    fill={buttonRight ? activeColor : inactiveColor}
                />
                {/* button_left */}
                <circle
                    className="button_left"
                    cx="400"
                    cy="161.3"
                    r="18.8"
                    fill={buttonLeft ? activeColor : inactiveColor}    
                />
                {/* button_up */}
                <circle 
                    className="button_up"
                    cx="438.1"
                    cy="123.2"
                    r="18.8"
                    fill={buttonUp ? activeColor : inactiveColor}    
                />
            </g>

            {/* D-PAD */}

            <g>
                <rect 
                    className="direction_up" 
                    x="207.1" 
                    y="220.5" 
                    width="18" 
                    height="22.5" 
                    rx="5"
                    ry="5"
                    fill={directionUp ? activeColor : inactiveColor}
                />
                <rect 
                    className="direction_down"
                    x="207.1" 
                    y="266.5" 
                    width="18" 
                    height="22.5" 
                    rx="5" 
                    ry="5"
                    fill={directionDown ? activeColor : inactiveColor}
                />
                <rect
                    className="direction_left"
                    x="181"
                    y="245.1"
                    width="22.5" 
                    height="18"
                    rx="5" 
                    ry="5"
                    fill={directionLeft ? activeColor : inactiveColor}
                />
                <rect
                    className="direction_right"
                    x="228.2"
                    y="245.1"
                    width="22.5"
                    height="18"
                    rx="5"
                    ry="5"
                    fill={directionRight ? activeColor : inactiveColor}
                />
            </g>

            {/* JOYSTICKS */}

            <g>
                <circle 
                    className="analog_left"
                    cx="142.1"
                    cy="159.8"
                    r="40"
                    fill={analogLeft ? activeColor : inactiveColor}
                    style={{
                        position: "relative",
                        transition: "transform 200ms ease-out",
                        transform:
                            analogLeftDirection.length > 0
                                ? `${createTransform(analogLeftDirection[0])} ${createTransform(
                                    analogLeftDirection[1]
                                )}`
                                : ""
                    }}
                />
                <circle 
                    className="analog_right"
                    cx="365.6"
                    cy="250"
                    r="40"
                    fill={analogRight ? activeColor : inactiveColor}
                    style={{
                        position: "relative",
                        transition: "transform 200ms ease-out",
                        transform:
                            analogRightDirection.length > 0
                                ? `${createTransform(analogRightDirection[0])} ${createTransform(
                                    analogRightDirection[1]
                                )}`
                                : ""
                    }}
                />
            </g>


            {/* SELECT & START */}
            {/* PLUS & MINUS */}

            {/* Plus */}
            <g>
                <rect 
                    className="start" 
                    x="327.9"
                    y="151.1"
                    width="8.6"
                    height="20.4"
                    fill={start ? activeColor : inactiveColor}
                />
                
                <rect 
                    className="start"
                    x="322"
                    y="157.1"
                    width="20.4"
                    height="8.6"
                    fill={start ? activeColor : inactiveColor}
                />
            </g>
            {/* Minus */}
            <rect
                className="select"
                x="238.9"
                y="157"
                width="20.4"
                height="8.6"
                fill={select ? activeColor : inactiveColor}    
            />

            {/* BUMPERS */}
            
            <g>
                <path
                    className="bumper_left"
                    d="M190.8,45.7c-10.4-15.6-115.3,21-103,34.2"
                    fill={bumperLeft ? activeColor : inactiveColor}
                />
                <path
                    className="bumper_right"
                    d="M387.1,48.2c-2.4-17.9,104.2,12.1,103.7,30.8"
                    fill={bumperRight ? activeColor : inactiveColor}
                />
            </g>

            {/* TRIGGERS */}

            <g>
            <rect 
                className="cls-1"
                x="130"
                width="30"
                height="40"
                fill={inactiveColor}
                style={{
                    transform: "scaleY(1)",
                    fill: "url(#leftTriggerGradient)"
                }}
            />
            <rect 
                className="cls-4"
                x="416.6"
                width="30"
                height="40"
                fill={inactiveColor}
                style={{
                    transform: "scaleY(1)",
                    fill: "url(#rightTriggerGradient)"
                }}
            />
        </g>

            {/* Gradients for Triggers */}

            {/* Left */}
            <linearGradient spreadMethod="reflect" y2="0%" x2="100%" y1="100%" x1="100%" id="leftTriggerGradient">
                <stop stopOpacity="1" stopColor={activeColor} offset={`${triggerLeft * 100}%`}/>
                <stop stopOpacity="1" stopColor={inactiveColor} offset={`${triggerLeft * 100}%`}/>
            </linearGradient>
            {/* Right */}
            <linearGradient spreadMethod="reflect" y2="0%" x2="100%" y1="100%" x1="100%" id="rightTriggerGradient">
                <stop stopOpacity="1" stopColor={activeColor} offset={`${triggerRight * 100}%`}/>
                <stop stopOpacity="1" stopColor={inactiveColor} offset={`${triggerRight * 100}%`}/>
            </linearGradient>

        </svg>
    );
}

export default GamepadSvg;
