import {useState} from "react";
import "./sidebar.css";
import React from "react";
import { widgets } from '../widgets/widget_space.tsx'
import {Widget} from '../widgets/widgets.tsx'

function SideBar() {
    const [sidebarActive, showSideBar] = useState(false);

    const toggleSideBar = () => showSideBar(!sidebarActive);

    return (
        <div className="sidebar-filter">
            <button className="taskbar-button" onClick={toggleSideBar}>
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
                    {widgets.map((widget) => {
                        return (
                            <div id={widget.title} 
                            draggable={true} 
                            className="widget" 
                            onDragStart={(event) => {
                                event.dataTransfer.setData("text", widget.title);
                            }}>
                                <Widget title={widget.title} data={<div />}/>
                            </div>
                        )
                    })}
                </div>
            </div>
        </div>
    );
}

export default SideBar