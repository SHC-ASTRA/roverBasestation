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

type Props = {
  item: WidgetType
  isOpacityEnabled?: boolean
  isDragging?: boolean
} & HTMLAttributes<HTMLDivElement>

export const Widget = forwardRef<HTMLDivElement, Props>(
  ({ item, isOpacityEnabled, isDragging, style, ...props }, ref) => {
    const styles: CSSProperties = {
      opacity: isOpacityEnabled ? "0.4" : "1",
      cursor: isDragging ? "grabbing" : "grab",
      lineHeight: "3.5",
      transform: isDragging ? "scale(1.05)" : "scale(1)",
      ...style,
    };

    return (
        <div ref={ref} style={styles} {...props}>
            <div className="widget" style={{
            borderRadius: "8px",
            boxShadow: isDragging
                ? "none"
                : "rgb(63 63 68 / 5%) 0px 0px 0px 1px, rgb(34 33 81 / 15%) 0px 1px 3px 0px",
            maxWidth: "100%",
            objectFit: "cover"
            }}>
                
                <div className="widget-title">
                    {item.title}
                </div>
                <div className="widget-content">
                    {item.data}
                </div>
            </div>
        </div>
    );
  }
);

export const SortableWidget = ({item, ...props}: Props) => {
    const { attributes, isDragging, listeners, setNodeRef, transform, transition } = useSortable({
        id: item.id,
    })

    const styles = {
        transform: CSS.Transform.toString(transform),
        transition: transition || undefined,
    }

    return (
        <Widget
            item={item}
            ref={setNodeRef}
            isOpacityEnabled={isDragging}
            style={styles}
            {...props}
            {...attributes}
            {...listeners}
        />
    )
    
}
