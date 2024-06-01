// base react
import React, { useState, useEffect } from "react"
import "../../node_modules/react-grid-layout/css/styles.css";
import "../../node_modules/react-resizable/css/styles.css";

import Responsive, {WidthProvider} from "react-grid-layout"
import {LayoutItem} from "react-grid-layout"

import { Widget } from "./widgets.tsx"

// component imports
import TestbedControl from "../components/testbedMotorControl.tsx"
import {CurrentTime} from "../components/time.tsx"
import { CoreControl } from "../components/core/CoreControl.tsx";
import { Feedback } from "../components/feedback/Feedback.tsx";
import { Map } from "../components/auto/Map.tsx";
import { FanControl } from "../components/bio/FanControl.tsx";
import { FaerieControl } from "../components/bio/FaerieControl.tsx";
import { FaerieSensors } from "../components/bio/FaerieSensors.tsx";
import { ArmPos } from "../components/arm/ArmPos.tsx";
import { ArmControl } from "../components/arm/ArmControl.tsx";
import { ChemicalDispersion } from "../components/bio/ChemicalDispersion.tsx";
import { BioArm } from "../components/bio/BioArm.tsx";
import CameraData from "../components/cameras/cameraFeed.tsx";
import { AutonomyControl } from "../components/auto/AutonomyControl.tsx";
import { Telemetry } from "../components/core/CoreRawTelemetry.tsx";

const ReactGridLayout = WidthProvider(Responsive);

function getWindowDimensions() {
    const { innerWidth: width, innerHeight: height } = window;
    return {
        width,
        height
    };
}
  
export default function useWindowDimensions() {
    const [windowDimensions, setWindowDimensions] = useState(getWindowDimensions());

    useEffect(() => {
        function handleResize() {
            setWindowDimensions(getWindowDimensions());
        }

        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    return windowDimensions;
}

export type WidgetData = {
    title: string
    data: JSX.Element
    width?: number
    height?: number
    minW?: number
    minH?: number
}

// Create a list of feedback topic titles
let feedbackWidgets = [
    {
        title: "Core Feedback",
        topic: "/core/feedback"
    },
    {
        title: "CITADEL Feedback",
        topic: "/bio/feedback"
    },
    {
        title: "Arm Feedback",
        topic: "/arm/feedback"
    },
    {
        title: "Autonomy Feedback",
        topic: "/auto/feedback"
    }
]
// Widgets that are shown in the sidebar
export let widgets: WidgetData[] = [
    {
        title: "Visual Gamepad",
        data: <TestbedControl controllerScale={2/3}/>,
    },
    {
        title: "Current Time",
        data: <CurrentTime/>,
    },
    {
        title: "Core Control",
        data: <CoreControl />,
        height: 3,
        minH: 3
    },
    {
        title: "Map",
        data: <Map />,
        minH: 8,
        minW: 5,
        height: 8,
        width: 5
    },
    {
        title: "CITADEL Bio Arm",
        data: <BioArm />,
        minW: 3,
        width: 3
    },
    {
        title: "CITADEL Fan Control",
        data: <FanControl />,
        width: 3,
        minW: 3,
        height: 4,
        minH: 4
    },
    {
        title: "CITADEL Chemical Dispersion",
        data: <ChemicalDispersion />,
        width: 4,
        minW: 4,
        height: 5,
        minH: 5
    },
    {
        title: "FAERIE Control",
        data: <FaerieControl />,
        width: 4,
        minW: 4,
        height: 7,
        minH: 7
    },
    {
        title: "FAERIE Sensor Data",
        data: <FaerieSensors />
    },
    {
        title: "Arm Control",
        data: <ArmControl />,
        minW: 3,
        width: 3,
        minH: 5,
        height: 5
    },
    {
        title: "Autonomy Control",
        data: <AutonomyControl />,
        minW: 6,
        width: 6,
        minH: 4,
        height: 4,
    },
    {
        title: "Camera",
        data: <CameraData defaultTopic={'/camera0/image_raw/compressed'}/>,
        minW: 3,
        width: 3,
        minH: 6,
        height: 6
    },
     {
        title: "Core Telemetry",
        data: <Telemetry />,
        height: 7,
        minH: 7,
        width: 4,
        minW: 4,
    },
    // Perform a form of "list comprehension" adapted to Typescript / Javascript functionality
    ...feedbackWidgets.map((widgetData) => {
        return({
            title: widgetData.title,
            data: <Feedback topicName={widgetData.topic}/>,
            // Width
            minW: 2,
            width: 4,
            // Height
            minH: 4,
            height: 4
        })
    })
];

widgets.sort((a, b) => a.title > b.title ? 1 : -1);

type WidgetSpaceProps = {
    props?: JSX.ElementAttributesProperty,
    staticWidgets: boolean,
}

type WidgetSpaceState = {
    layout: LayoutItem,
}

export class WidgetSpace extends React.PureComponent<WidgetSpaceProps, WidgetSpaceState> {
    static staticLayout: LayoutItem[] = [];

    constructor(props) {
        super(props);
        this.state = {
            layout: WidgetSpace.staticLayout
        }
        this.onLayoutChange = this.onLayoutChange.bind(this);
        this.onDrop = this.onDrop.bind(this);
        this.onRemove = this.onRemove.bind(this);

    }

    onLayoutChange(layout_: LayoutItem[]) {
        this.setState({ layout: layout_ });
        WidgetSpace.staticLayout = layout_;
    }

    isInLayout(widget) {
        for (let i = 0; i < this.state.layout.length; i++) {
            if (this.state.layout[i].i == widget.title) {
                return true;
            }
        }
        return false;
    }

    generateDOM() {
        return widgets.map((widget) => {
            if (this.isInLayout(widget)) {
                return (
                    <div key={widget.title} className="widget">
                        <Widget title={widget.title} data={widget.data} parent_space={this}/>
                    </div>
                )
            }
        })
    }

    onDrop(layout_, layoutItem, event) {
        event.preventDefault();
        const widgetTitle: string = event.dataTransfer.getData("text");

        let widget: WidgetData = {
            title: widgetTitle,
            data: <div/>
        };
        for (let i = 0; i < widgets.length; i++){
            if (widgets[i].title == widgetTitle) {
                widget = widgets[i];
                break;
            }
        }
        let item: LayoutItem = {
            i: widgetTitle,
            x: layoutItem.x,
            y: layoutItem.y,
            w: widget.width ? widget.width : 2,
            h: widget.height ? widget.height : 2,
            minW: widget.minW ? widget.minW : 2,
            minH: widget.minH ? widget.minH : 2,
        }
        layout_.push(item);

        this.onLayoutChange(layout_);      
    }

    onRemove(remove_widget, camera_topic="") {
        if(camera_topic) socket.emit('camera_close', camera_topic);
        // Set the state to the layout with the widget filtered out
        this.setState({ layout: this.state.layout.filter(widget => widget.i != remove_widget.title)} );
        // Perform some state updates
        WidgetSpace.staticLayout = this.state.layout;
    }

    render() {
        return (
            <ReactGridLayout
                className="layout"
                layout={this.state.layout}
                cols={12}
                rowHeight={70}
                width={1200}
                height={2400}
                onLayoutChange={this.onLayoutChange}
                verticalCompact={true}
                isDroppable={true}
                onDrop={this.onDrop}
                // Static widgets switch disables resizing and dragging and dropping
                isDraggable={!this.props.staticWidgets}
                resizeHandles={!this.props.staticWidgets ? ['se', 's', 'e'] : []}
            >
                {this.generateDOM()}
            </ReactGridLayout>
        );
    }
    
}