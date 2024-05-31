import React, {useState, useEffect} from 'react'

export const ChemicalDispersion = ({
    topicName = '/bio/control'
    }) => {

    return (
        <div> 
            <div className="button-wrapper">
                <p>Nominal Pump</p>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Nominal pump 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump1,${5000}`,
                    })
                })
            }}>Water</button>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Nominal pump 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump2,${1750}`,
                    })
                })
            }}>Meth. Blue</button>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Nominal pump 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump3,${5000}`,
                    })
                })
            }}>AA/BCA</button>
            </div>
            <div className="button-wrapper">
                <p>Short Pump</p>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Short pump 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump1,${500}`,
                    })
                })
            }}>Water</button>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Short pump 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump2,${50}`,
                    })
                })
            }}>Meth. Blue</button>
                <button className="round-button" style={{width: '6em', height: '4em'}} onClick={() => {
                console.log("Short pump 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `pump3,${500}`,
                    })
                })
            }}>AA/BCA</button>
            </div>
            <div className="button-wrapper" style={{justifyContent: 'space-around'}}>
                <button className="red-button" onClick={() => {
                console.log("Stopping all pumps");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'shutdown',
                    })
                })
                }} style={{width: '6em', height: '4em'}}>Stop</button>
            </div>
        </div>
    )
}