import React, {useState, useEffect} from 'react';
import SideBar from './components/sidebar.tsx'
import { WidgetSpace } from './widgets/widget_space.tsx';
import "./App.css";

function Header() {
    return (
        <div className="header">
            <img className="astra-logo" src="../ASTRA_Logo.png" alt="ASTRA Logo"></img>
            <h1>SHC ASTRA</h1>
            <img className="shc-logo" src="../SHC_Logo.png" alt="Space Hardware Club Logo"></img>
        </div>
    )
}

function Taskbar() {
    return (
        <div className="taskbar">
            <SideBar />
        </div>
    )
}

function Body() {
    return (
        <div className="body-background">
            <div className="widget-space">
                <WidgetSpace />
            </div>  
        </div>
    )
}

function App() {

    return (
        <div className="layout">
            <Header />
            <Taskbar />
            <Body />
        </div>
    )
}

export default App