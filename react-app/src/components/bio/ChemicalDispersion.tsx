import React, {useState, useEffect} from 'react'

export const ChemicalDispersion = ({
    topicName = '/bio/control'
    }) => {

    const longPump = 3000;
    const shortPump = 250;

    return (
        <div> 
            <div className="button-wrapper">
                <p>Nominal Pump</p>
                <button className="round-button" onClick={() => {
                console.log("Nominal pump 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump1,${longPump}`,
                    })
                })
            }}>1</button>
                <button className="round-button" onClick={() => {
                console.log("Nominal pump 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump2,${longPump}`,
                    })
                })
            }}>2</button>
                <button className="round-button" onClick={() => {
                console.log("Nominal pump 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump3,${longPump}`,
                    })
                })
            }}>3</button>
            </div>
            <div className="button-wrapper">
                <p>Short Pump</p>
                <button className="round-button" onClick={() => {
                console.log("Short pump 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump1,${shortPump}`,
                    })
                })
            }}>1</button>
                <button className="round-button" onClick={() => {
                console.log("Short pump 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump2,${shortPump}`,
                    })
                })
            }}>2</button>
                <button className="round-button" onClick={() => {
                console.log("Short pump 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump2,${shortPump}`,
                    })
                })
            }}>3</button>
            </div>
            <div className="button-wrapper" style={{justifyContent: 'space-around'}}>
                <button className="red-button" onClick={() => {
                console.log("Performing a full pump");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump,1,1,1,${longPump}`,
                    })
                })
            }} style={{width: '6em', height: '4em'}}>Full Pump</button>
                <button className="red-button" onClick={() => {
                console.log("Stopping all pumps");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'StopPump',
                    })
                })
            }} style={{width: '6em', height: '4em'}}>Stop</button>
            </div>
        </div>
    )
}