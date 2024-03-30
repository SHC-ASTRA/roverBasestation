import {useState} from "react";
import "./sidebar.css";
import React from "react";
import { WidgetSpace } from '../widgets/widget_space.tsx'

function SideBar() {
    const [sidebarActive, showSideBar] = useState(false);

    const toggleSideBar = () => showSideBar(!sidebarActive);

    return (
        <div className="sidebar-filter">
            <button className="sidebar-button" onClick={toggleSideBar}>
                Widgets
            </button>
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
                </div>
            </div>
        </div>
    );
}

export default SideBar