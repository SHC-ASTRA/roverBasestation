import { DndContext, DragEndEvent, DragOverlay, DragStartEvent, PointerSensor, TouchSensor, closestCenter, useSensor, useSensors } from "@dnd-kit/core";
import { SortableContext, arrayMove, rectSortingStrategy } from "@dnd-kit/sortable";
import React from "react";
import { useMemo, useState } from "react";
import { SortableWidget, WidgetType, Widget } from "../widgets/widgets.tsx";
import VisualGamepad from '../components/VisualGamepad.tsx';

type ContainerType = {
    id: string,
    content: WidgetType[]
}

const testWidgets: Widget[] = []

export const WidgetSpace = () => {
    // const [containers, setContainers] = useState<ContainerType[]>([]);
  
    const [items, setItems] = useState<Widget[]>(testWidgets);
  
    // for drag overlay
    const [activeItem, setActiveItem] = useState<Widget>();
  
    // for input methods detection
    const sensors = useSensors(useSensor(PointerSensor), useSensor(TouchSensor));
  
    // triggered when dragging starts
    const handleDragStart = (event: DragStartEvent) => {
      const { active } = event;
      setActiveItem(items.find((item) => item.state.id === active.id));
    }
  
    // const isInsideContainer = (over: any) => {
    //   // Check if the over object is inside the container
    //   return over.containerId === "box";
    // };
  
    // triggered when dragging ends
    const handleDragEnd = (event: DragEndEvent) => {
      const { active, over } = event;
      if (!over) return;
  
      const activeItem = items.find((item) => item.state.id === active.id);
      const overItem = items.find((item) => item.state.id === over.id);
  
      // Check if the card is outside the container
      // const isOutsideContainer = !isInsideContainer(over);
  
      // if (isOutsideContainer && activeItem) {
      //   // Remove the card from the state
      //   setItems((prev) => prev.filter((item) => item.id !== activeItem.id));
      // } else {
        // Reorder cards
        const activeIndex = items.findIndex((item) => item.state.id === active.id);
        const overIndex = items.findIndex((item) => item.state.id === over.id);
  
        if (activeIndex !== overIndex) {
          setItems((prev) => arrayMove<Widget>(prev, activeIndex, overIndex));
        }
      //}
  
      if (!activeItem || !overItem) {
        return
      }
  
      setActiveItem(undefined);
    }
  
    const handleDragCancel = () => {
      setActiveItem(undefined);
    }

    let item_ids: number[] = [];

    for (let widget of )

    return (
        <div className="widget-space">
        <DndContext
        sensors={sensors}
        collisionDetection={closestCenter}
        onDragStart={handleDragStart}
        onDragEnd={handleDragEnd}
        onDragCancel={handleDragCancel}
        >
            {/* <div> */}
            <SortableContext items={items} strategy={rectSortingStrategy}>
                
            <div style={{
            display: "grid",
            gridTemplateColumns: `repeat(6, 1fr)`,
            gridGap: 16,
            margin: "16px auto 20px"
            }} 
            >
                {items.map((item) => (
                <SortableWidget key={item.state.id} item={item} />
            ))}
            </div>
            </SortableContext>

            <DragOverlay adjustScale style={{ transformOrigin: "0 0 " }}>
            {activeItem ? <Widget item={activeItem} isDragging /> : null}
            </DragOverlay>
        </DndContext>
        </div>
      );
}