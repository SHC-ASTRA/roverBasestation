import React, {useState} from 'react'
import Slider from 'rc-slider';
import 'rc-slider/assets/index.css'
import Toggle from 'react-toggle';
import "react-toggle/style.css"

export const FaerieControl = ({
    topicName = '/arm/bio/control'
    }) => {
    const [motorSpeed, setMotorSpeed] = useState<number>(0);
    const [textInput, setText] = useState<number>(0);

    return (
        <div>
            <div style={{padding: '0.7em 2em 0.7em 2em'}}>
                <h6>Motor Speed</h6>
                <Slider dotStyle={{width: '10em', height: '10em'}} min={0} max={100} step={1} defaultValue={50} onChangeComplete={(value) => {
                    if (typeof value === "number") { // i love union types i love union types
                        value -= 50
                        console.log("Motor speed set to " + value * 2 + "%");
                        value /= 100
                        setMotorSpeed(value);
                        
                        fetch(topicName, {
                            method: 'POST',
                            headers: {
                                'Accept': 'application/json',
                                'Content-Type': 'application/json',
                            },
                            body: JSON.stringify({
                                command: "faerie,ctrl,duty," + value,
                            })
                        })
                    } 
                }}/>
            </div>
            <div style={{padding: '1em'}}>
                <input type="text" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 100 || value < -100) return;
                    setText(value);
                }}></input>
            
                <input type="submit" onClick={() => {
                    let value: number = textInput;
                    console.log("Motor speed set to " + value + "%");
                    value /= 200
                    setMotorSpeed(value);
                    
                    fetch(topicName, {
                        method: 'POST',
                        headers: {
                            'Accept': 'application/json',
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            command: "faerie/ctrl/duty/" + value,
                        })
                    })
                }}></input>
            </div>

            <button className="red-button" onClick={() => {
                console.log("Stopping the motor");
                fetch(topicName, {
                    method: 'POST',
                    headers: {
                        'Accept': 'application/json',
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        command: 'faerie/ctrl/stop',
                    })
                })
            }} style={{width: '8em', height: '4em'}}>Stop Motor</button>

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
                            command: 'faerie/ctrl/laser/' + toggle,
                        })
                    })
                }}/>
            </div>
        </div>
        
    )
}
