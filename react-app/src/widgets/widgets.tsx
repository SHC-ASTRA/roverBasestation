import React, {MouseEventHandler} from "react";
import { FC, forwardRef, HTMLAttributes, CSSProperties } from "react";

import "./widgets.css";

// DO NOT ASSIGN ID TO 0
// THE WIDGET WILL BE FUCKED
// DON'T ASK ME WHY
type WidgetProps = {
    title: string
    data: JSX.Element
    isOpacityEnabled?: boolean
    isDragging?: boolean
    handleDelete?: MouseEventHandler<HTMLSpanElement>
} & HTMLAttributes<HTMLDivElement>

export const Widget = forwardRef<HTMLDivElement, WidgetProps>(({title, data, isDragging, isOpacityEnabled, handleDelete, style, ...props}, ref) => {

    // CSS styles based on props, particularly isDragging
    const styles: CSSProperties = {
        cursor: isDragging ? "grabbing" : "grab",
        lineHeight: "3.5",
        transform: isDragging ? "scale(1.05)" : "scale(1)",
        ...style,
    };

    return (
        <div ref={ref} style={styles} {...props}>
            {/* wrapper widget. this is the thing that actually moves */}
            <div style={{
                maxWidth: "100%",
                objectFit: "cover",
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