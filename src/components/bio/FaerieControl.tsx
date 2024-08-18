import React, {useState} from 'react'
import Slider from 'rc-slider';
import 'rc-slider/assets/index.css'
import Toggle from 'react-toggle';
import "react-toggle/style.css";
import "./FaerieControl.css";

export const FaerieControl = ({
    topicName = '/arm/bio/control'
    }) => {
    const [motorSpeed, setMotorSpeed] = useState<number>(0);
    const [textInput, setText] = useState<number>(0);
    const [lastCommand, setLastCommand] = useState<string>("");

    const duty_cycle_command = 'faerie,ctrl,duty'
    const laser_command = 'faerie,ctrl,laser'
    const shake_command = 'faerie,ctrl,shake'
    return (
        <div>
            <div style={{padding: '0.7em 2em 0.7em 2em'}}>
                <h6>Motor Speed</h6>
                <label>
                    Motor Speed Slider:
                </label>
                <span>{motorSpeed}</span>
                <input type="range" list="motor-speed-range" min={-0.5} max={0.5} step={0.001} defaultValue={0} onMouseUp={(event) => {
                    let value: number = Number(event.currentTarget.value);
                    // Convert value down to scale for ROS transport to Faerie node
                    // on a scale from -0.5 to 0.5
                    console.log(value);
                    setMotorSpeed(value);
                    // if(value) 
                }}/>
                <datalist id="motor-speed-range">
                    <option value="-0.5" /* label="-50%" */></option>
                    <option value="-0.25" /* label="-25%" */></option>
                    <option value="-0.02" /* label="-2%" */></option>
                    <option value="0" label="0%"></option>
                    <option value="0.02" /* label="2%" */></option>
                    <option value="0.25" /* label="25%" */></option>
                    <option value="0.5" /* label="50%" */></option>
                </datalist>

            </div>
            <div style={{padding: '1em'}}>
                <input type="text" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 50 || value < -50) return;
                    setText(value);
                }}></input>
            
                <input type="submit" onClick={() => {
                    let value: number = textInput;
                    console.log("Motor speed set to " + value + "%");
                    value /= 100
                    setMotorSpeed(value);

                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: `${duty_cycle_command},${value}`,
                        })
                    })
                }}></input>
            </div>

            <button className="red-button" onClick={() => {
                console.log("Shaking SCABBARD open");

                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `${shake_command},open`,
                    })
                })
                
            }} style={{width: '8em', height: '4em', backgroundColor: "blue"}}>Shake Open</button>
            {// hehe red button with bg color blue
            // get fucked alex - jamie
            }

            <button className="red-button" onClick={() => {
                console.log("Stopping the motor");

                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `${duty_cycle_command},0`,
                    })
                })
            
            }} style={{width: '8em', height: '4em'}}>Stop Motor</button>

            <button className="red-button" onClick={() => {
                console.log("Shaking SCABBARD closed");

                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: `${shake_command},close`,
                    })
                })
        
            }} style={{width: '8em', height: '4em', backgroundColor: "blue"}}>Shake Closed</button>

            <div style={{paddingTop: '2em'}}>
                <h6>Laser Toggle</h6>
                <Toggle defaultChecked={false} onChange={(e) => {
                    const toggle: string = e.target.checked ? "on" : "off";
                    console.log("Setting the lasers to be " + toggle);

                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: `${laser_command},${toggle}`,
                        })
                    })
                    
                }}/>
            </div>
        </div>
        
    )
}
