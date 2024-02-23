import React, { useState } from "react";

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
        <svg viewBox={`0 0 ${1100/scale} ${500/scale}`} fill="none" {...props}>
            {/* Controller body */}

            <path
                className="background"
                d="M 505.765 150.961 C 489.51 140.569 501.237 134.633 484.412 121.769 C 467.587 108.905 399.308 87.13 387.429 97.026 C 375.55 106.922 362.196 108.899 362.196 108.899 L 289.962 108.899 L 217.844 108.899 C 217.844 108.899 204.484 106.922 192.611 97.026 C 180.738 87.13 112.451 108.899 95.628 121.769 C 78.804 134.633 90.53 140.57 74.275 150.961 C 58.02 161.353 15.467 304.843 15.467 304.843 C 15.467 304.843 -39.95 464.666 59.011 483.963 C 59.011 483.963 83.252 468.626 104.036 443.883 C 111.814 434.623 150.401 373.001 170.971 373.001 L 404.938 373.001 C 423.452 373.001 470.961 437.892 475.998 443.883 C 496.781 468.626 521.022 483.963 521.022 483.963 C 619.983 464.666 564.566 304.843 564.566 304.843 C 564.566 304.843 522.02 161.347 505.765 150.961 Z M 438.047 148.335 C 451.775 148.335 462.937 159.504 462.937 173.225 C 462.937 186.946 451.775 198.115 438.047 198.115 C 424.319 198.115 413.157 186.952 413.157 173.225 C 413.157 159.498 424.319 148.335 438.047 148.335 Z M 399.932 186.433 C 413.653 186.433 424.822 197.596 424.822 211.323 C 424.822 225.05 413.66 236.213 399.932 236.213 C 386.204 236.213 375.041 225.044 375.041 211.323 C 375.041 197.602 386.204 186.433 399.932 186.433 Z M 332.146 195.398 C 340.928 195.398 348.07 202.546 348.07 211.328 C 348.07 220.11 340.928 227.252 332.146 227.252 C 323.364 227.252 316.221 220.11 316.221 211.328 C 316.221 202.546 323.364 195.398 332.146 195.398 Z M 142.139 259.414 C 115.077 259.414 93.056 237.394 93.056 210.331 C 93.056 183.269 115.076 161.255 142.139 161.255 C 169.202 161.255 191.215 183.269 191.215 210.331 C 191.215 237.394 169.201 259.414 142.139 259.414 Z M 256.399 316.807 C 256.399 318.496 255.028 319.868 253.339 319.868 L 230.891 319.868 L 230.891 342.322 C 230.891 344.011 229.52 345.382 227.831 345.382 L 203.596 345.382 C 201.907 345.382 200.536 344.012 200.536 342.322 L 200.536 319.868 L 178.088 319.868 C 176.399 319.868 175.028 318.497 175.028 316.807 L 175.028 292.572 C 175.028 290.884 176.399 289.512 178.088 289.512 L 200.536 289.512 L 200.536 267.058 C 200.536 265.369 201.907 263.998 203.596 263.998 L 227.831 263.998 C 229.52 263.998 230.891 265.369 230.891 267.058 L 230.891 289.512 L 253.339 289.512 C 255.028 289.512 256.399 290.883 256.399 292.572 L 256.399 316.807 Z M 249.019 227.247 C 240.237 227.247 233.095 220.105 233.095 211.323 C 233.095 202.541 240.237 195.392 249.019 195.392 C 257.801 195.392 264.943 202.54 264.943 211.323 C 264.943 220.106 257.794 227.247 249.019 227.247 Z M 365.299 348.974 C 338.236 348.974 316.222 326.954 316.222 299.892 C 316.222 272.829 338.236 250.815 365.299 250.815 C 392.361 250.815 414.375 272.829 414.375 299.892 C 414.375 326.954 392.361 348.974 365.299 348.974 Z M 438.047 276.311 C 424.319 276.311 413.157 265.142 413.157 251.421 C 413.157 237.7 424.319 226.531 438.047 226.531 C 451.775 226.531 462.937 237.694 462.937 251.421 C 462.937 265.148 451.774 276.311 438.047 276.311 Z M 479.106 236.213 C 465.378 236.213 454.215 225.044 454.215 211.323 C 454.215 197.602 465.378 186.433 479.106 186.433 C 492.827 186.433 503.996 197.596 503.996 211.323 C 503.996 225.05 492.827 236.213 479.106 236.213 Z"
                fill="#C4C4C4"
            />

            {/* CIRCLE BUTTONS */}

            <circle
                className="button_down"
                cx={438.047}
                cy={251.421}
                r={18.77}
                fill={buttonDown ? activeColor : inactiveColor}
            />
            <circle
                className="button_right"
                cx={479.106}
                cy={211.323}
                r={18.77}
                fill={buttonRight ? activeColor : inactiveColor}
            />
            <circle
                className="button_left"
                cx={399.932}
                cy={211.323}
                r={18.77}
                fill={buttonLeft ? activeColor : inactiveColor}
            />
            <circle
                className="button_up"
                cx={438.047}
                cy={173.226}
                r={18.77}
                fill={buttonUp ? activeColor : inactiveColor}
            />

            {/* D-PAD */}

            <rect
                className="direction_up"
                x="206.961" y="270.493" width="18" height="22.5" rx="5" ry="5"
                fill={directionUp ? activeColor : inactiveColor}
            />
            <rect
                className="direction_down"
                x="206.961" y="316.516" width="18" height="22.5" rx="5" ry="5"
                fill={directionDown ? activeColor : inactiveColor}
            />
            <rect
                className="direction_left"
                x="180.855" y="295.076" width="22.5" height="18" rx="5" ry="5"
                fill={directionLeft ? activeColor : inactiveColor}
            />
            <rect
                className="direction_right"
                x="228.105" y="295.076" width="22.5" height="18" rx="5" ry="5"
                fill={directionRight ? activeColor : inactiveColor}
            />

            {/* JOYSTICKS */}

            <circle
                className="analog_left"
                cx={141.972}
                cy={209.777}
                r={40}
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
                cx={365.5}
                cy={299.996}
                r={40}
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


            {/* SELECT & START */}
            
            <circle
                className="select"
                fill={select ? activeColor : inactiveColor}
                cx="249.019" cy="211.323" r="9.804"
            />
            <circle
                className="start"
                cx="332.146" cy="211.323" r="9.804"
                fill={start ? activeColor : inactiveColor}
            />

            {/* BUMPERS */}
            <path
                className="bumper_left"
                d="M 190.651 95.67 C 180.285 80.047 75.382 116.624 87.641 129.907"
                fill={bumperLeft ? activeColor : inactiveColor}
            />
            <path
                className="bumper_right"
                d="M 493.317 126.916 C 482.995 142.539 378.539 105.962 390.745 92.679"
                fill={bumperRight ? activeColor : inactiveColor}
                style={{
                    transformBox: "fill-box",
                    transformOrigin: "50% 50%",
                    position: "relative",
                    // It needs mirrored and flipped for some reason
                    transform: "scaleY(-1) scaleX(-1)"
                }}
            />

            {/* TRIGGERS */}

            <rect x={129.907} y="50" width="30" height="40"
                className="trigger_left"
                fill={inactiveColor}
                style={{
                    transform: "scaleY(1)",
                    fill: "url(#leftTriggerGradient)"
                }}
            />
            <rect x={378.539+38} y="50" width="30" height="40"
                className="trigger_right"
                fill={inactiveColor}
                style={{
                    fill: "url(#rightTriggerGradient)"
                }}
            />

            {/* Gradients for Triggers */}

            {/* Left */}
            <linearGradient spreadMethod="reflect" y2="0%" x2="100%" y1="100%" x1="100%" id="leftTriggerGradient">
                <stop stop-opacity="1" stop-color={activeColor} offset={`${triggerLeft * 100}%`}/>
                <stop stop-opacity="1" stop-color={inactiveColor} offset={`${triggerLeft * 100}%`}/>
            </linearGradient>
            {/* Right */}
            <linearGradient spreadMethod="reflect" y2="0%" x2="100%" y1="100%" x1="100%" id="rightTriggerGradient">
                <stop stop-opacity="1" stop-color={activeColor} offset={`${triggerRight * 100}%`}/>
                <stop stop-opacity="1" stop-color={inactiveColor} offset={`${triggerRight * 100}%`}/>
            </linearGradient>

        </svg>
    );
}

export default GamepadSvg;
