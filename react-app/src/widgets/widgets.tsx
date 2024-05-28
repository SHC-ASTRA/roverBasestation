import React, {MouseEventHandler} from "react";
import { FC, forwardRef, HTMLAttributes, CSSProperties } from "react";

import "./widgets.css";
import { WidgetSpace } from "./widget_space.tsx";

// DO NOT ASSIGN ID TO 0
// THE WIDGET WILL BE FUCKED
// DON'T ASK ME WHY
type WidgetProps = {
    title: string
    data: JSX.Element
    isOpacityEnabled?: boolean
    isDragging?: boolean
    handleDelete?: MouseEventHandler<HTMLSpanElement>
    parent_space?: WidgetSpace
} & HTMLAttributes<HTMLDivElement>

export const Widget = forwardRef<HTMLDivElement, WidgetProps>(({title, data, isDragging, isOpacityEnabled, handleDelete, style, ...props}, ref) => {

    // CSS styles based on props, particularly isDragging
    const styles: CSSProperties = {
        cursor: isDragging ? "grabbing" : "grab",
        lineHeight: "3.5",
        transform: isDragging ? "scale(1.05)" : "scale(1)",
        ...style,
    };
    
    const removeStyle: CSSProperties = {
        position: "absolute",
        right: "2px",
        top: 0,
        cursor: "pointer"
      };

    function onRemoveItem(widget_title) {
        // If the parent_space prop has been provided,
        // and the widget is intended to be able to be removed
        if(props.parent_space) props.parent_space.onRemove(widget_title)
        // WILL NEED REWRITTEN FOR CAMERAS
    }

    return (
        <div ref={ref} style={styles} {...props}>
            {/* wrapper widget. this is the thing that actually moves */}
            <div style={{
                maxWidth: "100%",
                objectFit: "cover",
            }}>
                <div className="widget-title">
                    {title}
                    <span
                        className="removal-x"
                        style={removeStyle}
                        onClick={() => onRemoveItem({title})}
                    >
                        x
                    </span>
                </div>
                <div className="widget-content">
                    {data}
                </div>
            </div>
        </div>
    )
});