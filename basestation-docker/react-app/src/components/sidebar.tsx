import {useState} from "react";
import "./sidebar.css";
import "../widgets/widgets.tsx"
import React from "react";
import { WidgetSpace } from '../widgets/widget_space.tsx'

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
                    <WidgetSpace></WidgetSpace>
                </div>
            </div>
        </div>
    );
}

export default SideBar