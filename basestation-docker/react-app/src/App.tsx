import React, { useState, useEffect } from 'react';
// import SimpleGamepadDisplay from './components/simpleGamepad.tsx';
// import VisualGamepad from './components/VisualGamepad.tsx';
import TestbedControl from './components/testbedMotorControl.tsx';
import SideBar from './components/sidebar.tsx'
import "./App.css";

function Header() {
    return (
        <div className="header">
            <img className="logo" src="../ASTRA_Logo_512x512.png" alt="ASTRA Logo"></img>
            <h1>SHC ASTRA</h1>
            <SideBar />
        </div>
    )
}

function Body() {
    return (
        <div className="body-background">
            <div className="widget-space">
                {<TestbedControl/>}
            </div>

        </div>
    )
}

function App() {
    // useEffect(() => {
    //   fetch("/api").then(
    //     response => response.json()
    //   ).then(
    //     data => {
    //       setBackendData(data)
    //     }
    //   )
    // }, []);
    // [] is the list of dependencies

    return (
        <div className="layout">
            <Header />
            <Body />
            {/* <SimpleGamepadDisplay /> */}
            {/* <VisualGamepad scale={1/3}/> */}
            
        </div>
    )
}

export default App