import React, {useState, useEffect} from 'react';
import SideBar from './components/sidebar.tsx'
import { WidgetSpace } from './widgets/widget_space.tsx';
import Toggle from 'react-toggle'
import Dropdown from 'react-dropdown'
import 'react-dropdown/style.css';
import "./App.css";

function App() {
    const [staticWidgets, setStaticWidgets] = useState(false);

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
                <div className="toggle">
                    <Toggle checked={staticWidgets} onChange={(event) => {
                        setStaticWidgets(event.target.checked)
                    }}/>
                    <p>{staticWidgets ? "Live Mode" : "Edit Mode"}</p>
                </div>
                <SideBar />
            </div>
        )
    }

    function Body() {

        return (
            <div className="body-background">
                <div className="widget-space">

                    <WidgetSpace staticWidgets={staticWidgets} />
                </div>  
            </div>
        )
    }

    return (
        <div className="layout">
            <Header />
            <Taskbar />
            <Body />
        </div>
    )
}

export default App