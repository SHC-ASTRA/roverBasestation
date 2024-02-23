import React from "react";
import { forwardRef, HTMLAttributes, CSSProperties } from "react";

import { CSS } from "@dnd-kit/utilities";
import { useSortable } from "@dnd-kit/sortable";

import "./widgets.css";

// DO NOT ASSIGN ID TO 0
// THE WIDGET WILL BE FUCKED
// DON'T ASK ME WHY
export type WidgetType = {
    id: number,
    title: string,
    data: string | JSX.Element,
    container: string
}

type WidgetProps = {
    id: number,
    title: string,
    data: string | JSX.Element,
    container: string
    isOpacityEnabled?: boolean
    isDragging?: boolean
} & HTMLAttributes<HTMLDivElement>

export class Widget extends React.Component<WidgetProps, WidgetType> {
    constructor(props: WidgetProps) {
        super(props);

        this.state = {
            id: this.props.id,
            title: this.props.title,
            data: this.props.data,
            container: this.props.container
        }
    }

    updateData(data: string | JSX.Element) {
        this.setState({data: data});
    }

    render() {

        const styles: CSSProperties = {
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

let iterID = 1;

export const SortableWidget = ({...props}: WidgetProps) => {
    const widget_id = iterID;
    iterID++;

    const { attributes, isDragging, listeners, setNodeRef, transform, transition } = useSortable({
        id: widget_id,
    })

    const styles = {
        transform: CSS.Transform.toString(transform),
        transition: transition || undefined,
    }

    return (
        <Widget
            isOpacityEnabled={isDragging}
            {...props}
            {...attributes}
            {...listeners}
        />
    )
}