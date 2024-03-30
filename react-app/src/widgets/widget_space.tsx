// base react
import React, {FC, useState, useEffect} from "react"
import "../../node_modules/react-grid-layout/css/styles.css";
import "../../node_modules/react-resizable/css/styles.css";

import RGL, {WidthProvider} from "react-grid-layout"

import { Widget } from "./widgets.tsx"

// component imports
import TestbedControl from "../components/testbedMotorControl.tsx"
import {CurrentTime} from "../components/time.tsx"
import LiveData from "../components/liveData.tsx"

const ReactGridLayout = WidthProvider(RGL);

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

type WidgetData = {
    title: string
    data: JSX.Element
}

let widgets: WidgetData[] = [
    {
        title: "Visual Gamepad",
        data: <TestbedControl controllerScale={2/3}/>
    },
    {
        title: "Live Updating",
        data: <CurrentTime/>
    },
    {
        title: "Live Data",
        data: <LiveData topicName="/topic"></LiveData>
    }
];

let layout: any[] = [];

for (let i = 0; i < widgets.length; i++) {
    layout[i] = {
        i: widgets[i].title,
        x: (i * 2) % 12,
        y: Math.floor(i / 4),
        w: 2,
        h: 2,
        minW: 2,
        minH: 2
    }
}

export class WidgetSpace extends React.PureComponent<any, any> {

    constructor(props) {
        super(props);
        this.state = {
          layout: layout
        };
        this.onLayoutChange = this.onLayoutChange.bind(this);
    }

    onLayoutChange(layout) {
        this.setState({ layout: layout });
    }

    render() {
        return (
            <ReactGridLayout
                className="layout"
                layout={this.state.layout}
                cols={12}
                rowHeight={70}
                width={1200}
                isResizable="true"
                onLayoutChange={this.onLayoutChange}
                verticalCompact={false}
                resizeHandles={['se', 's', 'e']}
            >
                {/* {this.state.layout.map(el => this.createElement(el)} */}
                {widgets.map((widget) => {
                    return (
                        <div key={widget.title} className="widget">
                            <Widget title={widget.title} data={widget.data}/>
                        </div>
                    )
                })}
            </ReactGridLayout>
        );
    }
    componentDidMount() {
    // fetch data and set state
    }
}