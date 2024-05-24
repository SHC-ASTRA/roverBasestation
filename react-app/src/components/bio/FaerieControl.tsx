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
    const [lastCommand, setLastCommand] = useState<string>("");

    const duty_cycle_command = 'faerie,ctrl,duty,'
    const laser_command = 'faerie,ctrl,laser,'
    const stop_command = 'faerie,stop'

    function send_command_post(command, {toggleParam='off', valueParam=0}) {
        let command_to_send;
        
        // STOP COMMAND
        if(command.includes('stop')) {
            command_to_send = stop_command;
        }
        // LASER COMMAND
        else if(command.includes('laser')) {
            command_to_send = laser_command + toggleParam;
        }
        // DUTY CYCLE COMMAND
        else {
            command_to_send = duty_cycle_command + valueParam;
        }
        fetch(topicName, {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                command: command_to_send
            })
        })

        setLastCommand(command_to_send);
        // console.log(`Sent command ${command_to_send}`);
    }

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
                        
                        send_command_post(duty_cycle_command, {valueParam: value});
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
                    
                    send_command_post(duty_cycle_command, {valueParam: value});
                }}></input>
            </div>

            <button className="red-button" onClick={() => {
                console.log("Stopping the motor");
                
                send_command_post(stop_command, {});
            }} style={{width: '8em', height: '4em'}}>Stop Motor</button>

            <div style={{paddingTop: '2em'}}>
                <h6>Laser Toggle</h6>
                <Toggle defaultChecked={false} onChange={(e) => {
                    const toggle: string = e.target.checked ? "on" : "off";
                    console.log("Setting the lasers to be " + toggle);
                    
                    send_command_post(laser_command, {toggleParam: toggle});
                }}/>
            </div>

            <div>
                <span>Last Command: {lastCommand}</span>
            </div>
        </div>
        
    )
}
