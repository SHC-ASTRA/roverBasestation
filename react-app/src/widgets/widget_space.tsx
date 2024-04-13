// base react
import React, { MouseEventHandler } from "react"
import "../../node_modules/react-grid-layout/css/styles.css";
import "../../node_modules/react-resizable/css/styles.css";

import Responsive, {WidthProvider} from "react-grid-layout"
import {LayoutItem} from "react-grid-layout"

import { Widget } from "./widgets.tsx"

// component imports
import TestbedControl from "../components/testbedMotorControl.tsx"
import {CurrentTime} from "../components/time.tsx"
import LiveData from "../components/liveData.tsx"

const ReactGridLayout = WidthProvider(Responsive);

type WidgetData = {
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
        data: <CurrentTime/>
    },
    {
        title: "Topic Feedback",
        data: <LiveData topicName="/topic" />
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
                verticalCompact={true}
                isDroppable={true}
                onDrop={this.onDrop}
                resizeHandles={['se', 's', 'e']}
            >
                {this.generateDOM()}
            </ReactGridLayout>
        );
    }
    componentDidMount() {
    // fetch data and set state
    }
}