import React from 'react';
import SideBar from './components/sidebar.tsx'
import { WidgetSpace } from './widgets/widget_space.tsx';
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
                <WidgetSpace />
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
        </div>
    )
}

export default App