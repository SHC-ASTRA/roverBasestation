import React, { useState, useEffect, Component } from 'react';
import { Gamepad } from 'react-gamepad'

export class GamepadDisplayWidget extends Component {
    connectHandler(gamepadIndex) {
        console.log(`Gamepad ${gamepadIndex} connected !`)
      }
     
    disconnectHandler(gamepadIndex) {
        console.log(`Gamepad ${gamepadIndex} disconnected !`)
    }
    
    buttonChangeHandler(buttonName, down) {
        console.log(buttonName, down)
    }
    
    axisChangeHandler(axisName, value, previousValue) {
        console.log(axisName, value)
    }
    
    buttonDownHandler(buttonName) {
        console.log(buttonName, 'down')
    }
    
    buttonUpHandler(buttonName) {
        console.log(buttonName, 'up')
    }
        
    render() {
        return (
        {/* <Gamepad
            onConnect={this.connectHandler}
            onDisconnect={this.disconnectHandler}
    
            onButtonChange={this.buttonChangeHandler}
            onAxisChange={this.axisChangeHandler}
        /> */}
        )
    }

    // Gamepad status
    // Set gamepadStatus to defaults
    /* let gamepadStatus = {
        buttons: {
            a: false,
            b: false,
            x: false,
            y: false,
            left_bumper: false,
            right_bumper: false,
            left_trigger: false,
            right_trigger: false,
            view: false,
            menu: false,
            d_up: false,
            d_down: false,
            d_left: false,
            d_right: false
        },
        trigger_vals: {
            left_trigger_val: 0,
            right_trigger_val: 0
        },
        stick_vals: {
            left_hval: 0,
            left_vval: 0,
            right_hval: 0,
            right_vval: 0
        }
    }; */
}