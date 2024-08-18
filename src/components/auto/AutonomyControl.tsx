import React, {useState} from 'react';
import "./AutonomyControl.css"

type AutonomyModes = "Stop" | "Go To" | "ARUCO" | "Object Detection"

export const AutonomyControl = ({
    topicName='/auto/control'
}) => {

    const [gpsLat, setLat] = useState<number>(0);
    const [gpsLong, setLong] = useState<number>(0);
    const [period, setPeriod] = useState<number>(0);
    const [mode, setMode] = useState<AutonomyModes>("Stop");


    return (
        <>
            <div>Current Mode: {mode}</div>
            <div style={{display: 'flex', flexDirection: 'row', alignItems: "center", justifyContent: "space-evenly"}}>
                <input type="text" placeholder="GPS Latitude" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 180 || value < -180) return;
                    setLat(value);
                }}></input>
                <input type="text" placeholder="GPS Longitude" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 180 || value < -180) return;
                    setLong(value);
                }}></input>
                <input type="text" placeholder="Period" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 180 || value < -180) return;
                    setPeriod(value);
                }}></input>
            </div>
            <div style={{display: 'flex', flexDirection: 'row', alignItems: "center", justifyContent: "space-evenly"}}>
                <button className="control-button" id="control-button" onClick={() => {
                    console.log("Stopping the autonomy.");
                    setMode("Stop")
                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: 'Stop',
                        })
                    })
                }}>Stop</button>
                <button className="control-button" id="control-button" onClick={() => {
                    if (gpsLat === 0 || gpsLong === 0 || period === 0) return;
                    console.log("Going to the specified location.");
                    setMode('Go To')
                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: 'GoTo',
                            gpsLat: gpsLat,
                            gpsLong: gpsLong,
                            period: period
                        })
                    })
                }}>Go To</button>
                <button className="control-button" id="control-button" onClick={() => {
                    if (gpsLat === 0 || gpsLong === 0 || period === 0) return;
                    setMode("ARUCO")
                    console.log("Starting the ARUCO mission.");
                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: 'ARUCO',
                            gpsLat: gpsLat,
                            gpsLong: gpsLong,
                            period: period
                        })
                    })
                }}>ARUCO</button>
                <button className="control-button" id="control-button" onClick={() => {
                    if (gpsLat === 0 || gpsLong === 0 || period === 0) return;
                    console.log("Starting to detect objects.");
                    setMode("Object Detection")
                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: 'Object',
                            gpsLat: gpsLat,
                            gpsLong: gpsLong,
                            period: period
                        })
                    })
                }}>Object Detection</button>
            </div>
        </>
    )
}