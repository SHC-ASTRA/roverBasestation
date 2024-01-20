import {useState} from "react";
import "./sidebar.css";
import React from "react";

function SideBar() {
    const [sidebarActive, showSideBar] = useState(false);

    const toggleSideBar = () => showSideBar(!sidebarActive);

    return (
        <div className="sidebar-filter">
            <div className="sidebar-button" onClick={toggleSideBar}>
                <h2>Widgets</h2>
            </div>
            <div 
            className={
                sidebarActive ? "sidebar active" : "sidebar"
            }
            style={{
                width: sidebarActive 
                    ? window.innerWidth > 500 
                        ? 500 
                        : window.innerWidth 
                    : 0
            }}
            >
                <div className="sidebar-data">
                    <span className="hide-icon" onClick={toggleSideBar}>X</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                    <span>Hello!</span>
                </div>
            </div>
        </div>
    );
}

export default SideBar