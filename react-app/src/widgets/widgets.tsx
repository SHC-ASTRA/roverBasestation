import React from "react";
import { FC, forwardRef, HTMLAttributes, CSSProperties } from "react";
import { CSS } from "@dnd-kit/utilities";
import { useSortable } from "@dnd-kit/sortable";

import "./widgets.css";

// DO NOT ASSIGN ID TO 0
// THE WIDGET WILL BE FUCKED
// DON'T ASK ME WHY
type WidgetProps = {
    title: string
    data: JSX.Element
    isOpacityEnabled?: boolean
    isDragging?: boolean
} & HTMLAttributes<HTMLDivElement>

export const Widget = forwardRef<HTMLDivElement, WidgetProps>(({title, data, isDragging, isOpacityEnabled, style, ...props}, ref) => {

    // CSS styles based on props, particularly isDragging
    const styles: CSSProperties = {
        opacity: isDragging ? "0.4" : "1",
        cursor: isDragging ? "grabbing" : "grab",
        lineHeight: "3.5",
        transform: isDragging ? "scale(1.05)" : "scale(1)",
        ...style,
    };

    return (
        <div ref={ref} style={styles} {...props}>
            {/* wrapper widget. this is the thing that actually moves */}
            <div className="widget" style={{
                borderRadius: "8px",
                boxShadow: isDragging
                    ? "none"
                    : "rgb(63 63 68 / 5%) 0px 0px 0px 1px, rgb(34 33 81 / 15%) 0px 1px 3px 0px",
                maxWidth: "100%",
                objectFit: "cover"
            }}>
                <div className="widget-title">
                    {title}
                </div>
                <div className="widget-content">
                    {data}
                </div>
            </div>
        </div>
    )
});

// essentially acts as a wrapper around the widget with dragging capabilities that handles changes when drags start
export const SortableWidget: FC<WidgetProps> = ({title, data, ...props}) => {
    const {
        isDragging,
        attributes,
        listeners,
        setNodeRef,
        transform,
        transition
    } = useSortable({ id: title });
    // widgets are identified by their title. please make sure to name the titles different things when adding new widgets

    const style = {
        transform: CSS.Transform.toString(transform),
        transition: transition || undefined,
    };

    return (
        <Widget
            title={title}
            data={data}
            ref={setNodeRef}
            style={style}
            isOpacityEnabled={isDragging}
            {...props}
            {...attributes}
            {...listeners}
        />
    );
}