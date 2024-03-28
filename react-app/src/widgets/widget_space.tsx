// base react
import React, {FC, useState, useEffect} from "react"

// sensors
import {PointerSensor, TouchSensor, useSensors, useSensor, DndContext} from "@dnd-kit/core"

// dragging
import {DragStartEvent, DragEndEvent, closestCenter, DragOverlay} from "@dnd-kit/core"

// sorting
import {SortableContext, arrayMove, rectSortingStrategy} from "@dnd-kit/sortable"

// widgets
import {Widget, SortableWidget} from "./widgets.tsx"

// component imports
import TestbedControl from "../components/testbedMotorControl.tsx"
import {CurrentTime} from "../components/time.tsx"
import LiveData from "../components/liveData.tsx"

type WidgetData = {
    title: string
    data: JSX.Element
}

// once the widgets have titles and data put them here
// the title should be a string that goes on top of the widget
// the data should be a component that you make, import, and add here
let widgets = [
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

for (let i = 1; i <= 25; i++) {
    widgets.push({
        title: i.toString(),
        data: <div>{i}</div>
    })
}

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


export const WidgetSpace: FC = () => {
    const [items, setItems] = useState<WidgetData[]>(widgets);
    const [activeItem, setActiveItem] = useState<WidgetData>();
    const sensors = useSensors(useSensor(PointerSensor), useSensor(TouchSensor));
    const {height, width} = useWindowDimensions();

    const handleDragStart = (event: DragStartEvent) => {
        setActiveItem(items.find((item) => item.title === event.active.id));
    };

    const handleDragEnd = (event: DragEndEvent) => {
        const {active, over} = event;
        if (!over) return;

        const activeItem = items.find((item) => item.title === active.id);
        const overItem = items.find((item) => item.title === over.id);
  
        const activeIndex = items.findIndex((item) => item.title === active.id);
        const overIndex = items.findIndex((item) => item.title === over.id);

        if (activeIndex !== overIndex) {
            setItems((prev) => arrayMove<WidgetData>(prev, activeIndex, overIndex));
        }
  
        if (!activeItem || !overItem) return;

        setActiveItem(undefined);
    };

    const handleDragCancel = () => {
        setActiveItem(undefined);
    };

    return (
        <DndContext 
            sensors={sensors} 
            collisionDetection={closestCenter}
            onDragStart={handleDragStart}
            onDragEnd={handleDragEnd}
            onDragCancel={handleDragCancel}
        >

            <SortableContext items={items.map((item) => item.title)} strategy={rectSortingStrategy}>
                <div style={{
                display: "grid",
                gridTemplateColumns: `repeat(${(Math.floor(width / 267))}, 1fr)`,
                gridGap: 16,
                margin: "16px auto 20px"
                }} 
                >
                    {items.map((item) => (
                        <SortableWidget key={item.title} title={item.title} data={item.data}/>
                    ))}
                </div>

            </SortableContext>

            <DragOverlay adjustScale style={{ transformOrigin: "0 0 " }}>
            {activeItem ? <Widget title={activeItem.title} data={activeItem.data} isDragging /> : null}
            </DragOverlay>

        </DndContext>
    )
}