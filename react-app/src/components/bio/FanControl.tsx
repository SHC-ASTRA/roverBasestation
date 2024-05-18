import React, {useState, useEffect} from 'react'

export const FanControl = ({
    topicName = '/bio/control'
    }) => {

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
                        command: 'NFan1',
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
                        command: 'NFan2',
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
                        command: 'NFan3',
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
                        command: 'SFan1',
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
                        command: 'SFan2',
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
                        command: 'SFan3',
                    })
                })
            }}>3</button>
            </div>
        </div>
    )
}