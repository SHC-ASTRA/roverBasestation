// base react
import React, { MouseEventHandler, useState, useEffect} from "react"

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
        data: <CoreControl />
    },
    {
        title: "Core Feedback",
        data: <CoreFeedback />
    },
    {
        title: "Map",
        data: <Map />,
    }
];

const layout: LayoutItem[] = [];

export class WidgetSpace extends React.PureComponent<any, any> {

    constructor(props) {
        super(props);
        this.state = {
          layout: layout
        };
        this.onLayoutChange = this.onLayoutChange.bind(this);
    }

    onLayoutChange(layout_) {
        this.setState({ layout: layout_ });
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
        layout.push({    
            i: widgetTitle,
            x: layoutItem.x,
            y: layoutItem.y,
            w: widget.width ? widget.width : 2,
            h: widget.height ? widget.height : 2,
            minW: widget.minW ? widget.minW : 2,
            minH: widget.minH ? widget.minH : 2
        }) 
        this.onLayoutChange(layout);      
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
                verticalCompact={false}
                isDroppable={true}
                onDrop={this.onDrop}
                resizeHandles={['se', 's', 'e']}
            >
                {this.generateDOM()}
            </ReactGridLayout>
        );
    }
    
}