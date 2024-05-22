import React, {useState, useEffect, MouseEvent} from 'react'

export const BioArm = ({
    topicName = '/bio/control'
    }) => {

    // extend half
    // retract
    // reset
    // inc 2 deg
    // dec 2 deg

    return (
        <div className="button-wrapper">
            <button className="control-button" onClick={() => {
                console.log("Extending the arm.");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'servoHalf',
                    })
                })
            }}>
                Extend
            </button>
            <button className="control-button" onClick={() => {
                console.log("Retracting the arm.");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'servoFullRetract',
                    })
                })
            }}>
                Retract
            </button>
            <button className="control-button" onClick={() => {
                console.log("Shifting the arm up.");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'servoRetract',
                    })
                })
            }}>
                Shift Up
            </button>
            <button className="control-button" onClick={() => {
                console.log("Shifting the arm down.");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'servoExtend',
                    })
                })
            }}>
                Shift Down
            </button>
            <button className="control-button" onClick={() => {
                console.log("Resetting the Bio Arm.");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'servoReset',
                    })
                })
            }}>
                Reset
            </button>
        </div>
    )
}