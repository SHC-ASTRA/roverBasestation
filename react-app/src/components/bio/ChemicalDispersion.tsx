import React, {useState, useEffect} from 'react'

export const ChemicalDispersion = ({
    topicName = '/bio/control'
    }) => {
        
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
                        command: 'NPump1',
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
                        command: 'NPump2',
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
                        command: 'NPump3',
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
                        command: 'SPump1',
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
                        command: 'SPump2',
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
                        command: 'SPump3',
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
                        command: 'FPump',
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