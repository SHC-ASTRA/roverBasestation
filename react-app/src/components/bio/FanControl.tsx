import React, {useState, useEffect} from 'react'

export const FanControl = ({
    topicName = '/bio/control'
    }) => {

    const longFan = 15000;
    const shortFan = 3000;

    return (
        <div>
            <div className="button-wrapper">
                <p>Nominal Fan</p>
                <button className="round-button" onClick={() => {
                console.log("Nominal fan 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan1,${longFan}`,
                    })
                })
            }}>1</button>
                <button className="round-button" onClick={() => {
                console.log("Nominal fan 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan2,${longFan}`,
                    })
                })
            }}>2</button>
                <button className="round-button" onClick={() => {
                console.log("Nominal fan 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan3,${longFan}`,
                    })
                })
            }}>3</button>
            </div>
            <div className="button-wrapper">
                <p>Short Fan</p>
                <button className="round-button" onClick={() => {
                console.log("Short fan 1");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan1,${shortFan}`,
                    })
                })
            }}>1</button>
                <button className="round-button" onClick={() => {
                console.log("Short fan 2");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan2,${shortFan}`,
                    })
                })
            }}>2</button>
                <button className="round-button" onClick={() => {
                console.log("Short fan 3");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `fan3,${shortFan}`,
                    })
                })
            }}>3</button>
            </div>
        </div>
    )
}