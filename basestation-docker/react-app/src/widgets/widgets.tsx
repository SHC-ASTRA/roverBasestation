import React from "react";
import { forwardRef, HTMLAttributes, CSSProperties } from "react";

import { CSS } from "@dnd-kit/utilities";
import { useSortable } from "@dnd-kit/sortable";

import "./widgets.css";

// DO NOT ASSIGN ID TO 0
// THE WIDGET WILL BE FUCKED
// DON'T ASK ME WHY
type WidgetType = {
    id: number,
    title: string,
    data: string | JSX.Element,
    container: string
}

type WidgetProps = {
    title: string,
    data: string | JSX.Element,
    container: string
    isOpacityEnabled?: boolean
    isDragging?: boolean
} & HTMLAttributes<HTMLDivElement>

let iterID = 1;

export class Widget extends React.Component<WidgetProps, WidgetType> {
    constructor(props: WidgetProps) {
        super(props);

        this.state = {
            id: iterID,
            title: this.props.title,
            data: this.props.data,
            container: this.props.container
        }

        iterID++;
    }

    updateData(data: string | JSX.Element) {
        this.setState({data: data});
    }

    render() {
        const { attributes, isDragging, listeners, transform, transition } = useSortable({
            id: this.state.id,
        })

        const styles: CSSProperties = {
            transition: transition || undefined,
            opacity: this.props.isOpacityEnabled ? "0.4" : "1",
            cursor: this.props.isDragging ? "grabbing" : "grab",
            lineHeight: "3.5",
            transform: this.props.isDragging ? "scale(1.05)" : "scale(1)",
            ...this.props.style,
        };
    
        return (
            <div style={styles} {...this.props}>
                <div className="widget" style={{
                    borderRadius: "8px",
                    boxShadow: this.props.isDragging
                        ? "none"
                        : "rgb(63 63 68 / 5%) 0px 0px 0px 1px, rgb(34 33 81 / 15%) 0px 1px 3px 0px",
                    maxWidth: "100%",
                    objectFit: "cover"
                }}>
             
                    <div className="widget-title">
                        {this.state.title}
                    </div>
                    <div className="widget-content">
                        {this.state.data}
                    </div>
                </div>
            </div>
        );
    }
}
