import React, {useState, useEffect} from 'react';
import SideBar from './components/sidebar.tsx'
import { WidgetSpace } from './widgets/widget_space.tsx';
import Toggle from 'react-toggle'
import { ConnectionStatus } from './components/taskbar/ConnectionLatency.tsx'
import "./App.css";
import { useElapsedTime } from 'use-elapsed-time';

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
        const { elapsedTime, reset } = useElapsedTime({isPlaying: true});

        return (
            <div className="taskbar">
                <div style={{display:'flex', flexDirection:'row'}}>
                    <div style={{padding: '5px', margin: '5px'}}>{`Elapsed: ${new Date(elapsedTime * 1000).toISOString().substring(11, 19)}`}</div>
                    <button className='taskbar-button' onClick={() => {reset(0)}}>Reset Timer</button>
                </div>
                <ConnectionStatus />
                <div className="toggle" style={{display:'flex', flexDirection:'row'}}>
                    <Toggle checked={staticWidgets} style={{marginRight: '1em'}} onChange={(event) => {
                        setStaticWidgets(event.target.checked)
                    }}/>
                    <div className="side-text">{staticWidgets ? "Live Mode" : "Edit Mode"}</div>
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