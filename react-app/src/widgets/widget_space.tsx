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
import LiveData from "../components/liveData.tsx"
import { AutoFeedback } from "../components/auto/AutoFeedback.tsx";
import { CoreControl } from "../components/core/CoreControl.tsx";
import { CoreFeedback } from "../components/core/CoreFeedback.tsx";
import { Map } from "../components/auto/Map.tsx";
import { PumpStatus } from "../components/bio/PumpStatus.tsx";
import { FanControl } from "../components/bio/FanControl.tsx";
import { FaerieMotor } from "../components/bio/FaerieMotor.tsx";
import { FaerieLaser } from "../components/bio/FaerieLaser.tsx";
import { FaerieSensors } from "../components/bio/FaerieSensors.tsx";
import { ArmPos } from "../components/arm/ArmPos.tsx";
import { ArmControl } from "../components/arm/ArmControl.tsx";
import { ArmLaser } from "../components/arm/ArmLaser.tsx";
import { ChemicalDispersion } from "../components/bio/ChemicalDispersion.tsx";
import { BioArm } from "../components/bio/BioArm.tsx";
import CameraData from "../components/cameraFeed.tsx";

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
        title: "Live Data",
        data: <LiveData topicName="/topic"></LiveData>
    },
    {
        title: "Autonomy Feedback",
        data: <AutoFeedback/>,
    },
    {
        title: "Core Control",
        data: <CoreControl />,
        height: 3,
        minH: 3
    },
    {
        title: "Core Feedback",
        data: <CoreFeedback />
    },
    {
        title: "Map",
        data: <Map />,
        minH: 4,
        minW: 4
    },
    {
        title: "Bio Arm",
        data: <BioArm />,
        minW: 3,
        width: 3
    },
    {
        title: "Fan/Pump Status",
        data: <PumpStatus />
    },
    {
        title: "Fan Control",
        data: <FanControl />,
        width: 3
    },
    {
        title: "Chemical Dispersion",
        data: <ChemicalDispersion />,
        width: 3,
        minW: 3,
        height: 3,
        minH: 3
    },
    {
        title: "FAERIE Motor Speed",
        data: <FaerieMotor />
    },
    {
        title: "FAERIE Laser",
        data: <FaerieLaser />
    },
    {
        title: "Humidity/Temp Sensor Data",
        data: <FaerieSensors />
    },
    {
        title: "Arm Position",
        data: <ArmPos />
    },
    {
        title: "Arm Control",
        data: <ArmControl />,
        minW: 3,
        width: 3,
        minH: 3,
        height: 3
    },
    {
        title: "Arm Laser",
        data: <ArmLaser />
    },
    {
        title: "Camera 1",
        data: <CameraData defaultTopic={'/camera0/image_raw/compressed'}/>,
        minW: 3,
        width: 3,
        minH: 6,
        height: 6
    },
    {
        title: "Camera 2",
        data: <CameraData defaultTopic={'/camera1/image_raw/compressed'}/>
    },
    {
        title: "Camera 3",
        data: <CameraData defaultTopic={'/camera2/image_raw/compressed'}/>
    }
];

// const BiosensorPreset: LayoutItem[] = [
//     {i: "Bio Arm",
//     x: 0,
//     y: 0,
//     height: 3,
//     width: 3,
//     minW: 3,
//     minH: 3}
// ];

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
                        <Widget title={widget.title} data={widget.data}/>
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