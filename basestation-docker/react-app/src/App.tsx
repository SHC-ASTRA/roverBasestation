import React, { useState, useEffect } from 'react';
import SimpleGamepadDisplay from './components/simpleGamepad.tsx';
import VisualGamepad from './components/VisualGamepad.tsx';
import TestbedControl from './components/testbedMotorControl.tsx';
import SideBar from './components/sidebar.tsx'
import { WidgetSpace } from './widgets/widget_space.tsx';
import "./App.css";
import { DndContext } from '@dnd-kit/core';

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
            <WidgetSpace />
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
        </div>
    )
}

export default App