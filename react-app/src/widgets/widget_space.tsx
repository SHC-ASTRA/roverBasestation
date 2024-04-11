// base react
import React, {FC, useState, useEffect} from "react"
import "../../node_modules/react-grid-layout/css/styles.css";
import "../../node_modules/react-resizable/css/styles.css";

import RGL, {WidthProvider, Responsive} from "react-grid-layout"

import { Widget } from "./widgets.tsx"

// component imports
import TestbedControl from "../components/testbedMotorControl.tsx"
import {CurrentTime} from "../components/time.tsx"
import LiveData from "../components/liveData.tsx"
import { AutoFeedback } from "../components/auto/AutoFeedback.tsx";
import { CoreControl } from "../components/core/CoreControl.tsx";
import { CoreFeedback } from "../components/core/CoreFeedback.tsx";
import { Map } from "../components/auto/Map.tsx";


const ResponsiveReactGridLayout = WidthProvider(Responsive);

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

let widgets: WidgetData[] = [
    {
        title: "Visual Gamepad",
        data: <TestbedControl controllerScale={2/3}/>,
        width: 3,
        height: 3,
    },
    {
        title: "Live Updating",
        data: <CurrentTime/>,
        width: 3,
        height: 5
    },
    {
        title: "Live Data",
        data: <LiveData topicName="/topic"></LiveData>
    },
    {
        title: "Autonomy Feedback",
        data: <AutoFeedback/>,
        width: 2,
        height: 10
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
        width: 5,
        height: 10
    }
];

let layout: any[] = [];

// for (let i = 0; i < widgets.length; i++) {
//     layout[i] = {
//         i: widgets[i].title,
//         x: (i * 2) % 12,
//         y: Math.floor(i / 4),
//         w: widgets[i].width ? widgets[i].width : 2,
//         h: widgets[i].height ? widgets[i].height : 2,
//         minW: widgets[i].minW ? widgets[i].minW : 2,
//         minH: widgets[i].minH ? widgets[i].minH : 2,
//     }
// }
  
const initialLayout: any[] = widgets.map((widget, i) => ({
i: widgets[i].title,
    x: (i * 2) % 12,
    y: Math.floor(i / 4),
    w: widgets[i].width ? widgets[i].width : 2,
    h: widgets[i].height ? widgets[i].height : 2,
    minW: widgets[i].minW ? widgets[i].minW : 2,
    minH: widgets[i].minH ? widgets[i].minH : 2,
}));

const ToolboxItem: React.FC<{item: WidgetData; onTakeItem: (item:WidgetData) => void}> = ({ item, onTakeItem }) => (
    <div
      className="toolbox_item"
      onClick={() => onTakeItem(item)}
    >
      {item.title}
    </div>
)
  
// export const ToolBox: React.FC<{items: WidgetData[]; onTakeItem: (item:WidgetData) => void}> = ({ items, onTakeItem }) => (
//     <div className="toolbox">
//         <span className="toolbox_title">Toolbox</span>
//         <div className="toolbox_items">
//         {items.map((item, index) => (
//             <ToolboxItem key={index} item={item} onTakeItem={onTakeItem} />
//         ))}
//         </div>
//     </div>
// )
let toolboxItems: WidgetData[] = [];

export class WidgetSpace extends React.PureComponent<any, any> {

    // constructor(props) {
    //     super(props);
    //     this.state = {
    //       layout: layout
    //     };
    //     this.onLayoutChange = this.onLayoutChange.bind(this);
    // }

    static defaultProps = {
        className: "layout",
        rowHeight: 30,
        onLayoutChange: function() {},
        cols: { lg: 12, md: 10, sm: 6, xs: 4, xxs: 2 },
        initialLayout: initialLayout
    };

    state = {
        // compactType: "vertical",
        mounted: false,
        // layout: layout,
        layouts: { lg: this.props.initialLayout },
        toolbox: { lg: [] }
    };

    componentDidMount() {
        this.setState({ mounted: true });
    }

    onLayoutChange = (layout, layouts) => {
        this.props.onLayoutChange(layout, layouts);
        this.setState({ layouts });
    };
    
    onNewLayout = () => {
        this.setState({
            layouts: { lg: initialLayout }
        });
    };

    // onLayoutChange(layout) {
    //     this.setState({ layout: layout });
    // }

    onTakeItem = (item: WidgetData) => {
        this.setState(prevState => ({
            toolbox: {
              ...prevState.toolbox,
              [prevState]: prevState.toolbox[
                prevState
              ].filter(({ i }) => i !== item.title)
            },
            layouts: {
              ...prevState.layouts,
              [prevState]: [
                ...prevState.layouts[prevState],
                item
              ]
            }
        }))
        
    }

    onDrop = (layout, layoutItem, _event) => {
        alert(`Dropped element props:\n${JSON.stringify(layoutItem, ['x', 'y', 'w', 'h'], 2)}`);
    };

    render() {
        return (
            <div>
                <div>
                <div
                    className="droppable-element"
                    draggable={true}
                    unselectable="on"
                    onDragStart={e => e.dataTransfer.setData("text/plain", "")}
                    >
                    Droppable Element (Drag me!)
                </div>
                </div>
                
                <ResponsiveReactGridLayout
                    {...this.props}
                    layouts={this.state.layouts}
                    onLayoutChange={this.onLayoutChange}
                    measureBeforeMount={false}
                    useCSSTransforms={this.state.mounted}
                    resizeHandles={['se', 's', 'e']}
                    onDrop={this.onDrop}
                    >
                    {widgets.map((widget) => {
                        return (
                            <div key={widget.title} className="widget">
                                <Widget title={widget.title} data={widget.data}/>
                            </div>
                        )
                    })}
                </ResponsiveReactGridLayout>
            </div>
        );
    }
    
}