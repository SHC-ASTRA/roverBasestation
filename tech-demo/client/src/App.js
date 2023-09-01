import React, {useEffect, useState} from 'react';
import "./App.css";

function App() {
  const [backendData, setBackendData] = useState([{}]);
  const [widgets, setWidgets] = useState([]);

  const removeWidget = (index) => {
    console.log("removing widget...");
    setWidgets(widgets.filter((_, i) => i !== index));
  }

  function handleOnDrag(e, widgetType){
    e.dataTransfer.setData("widgetType", widgetType);
  }

  function handleOnDrop(e){
    const widgetType = e.dataTransfer.getData("widgetType");
    setWidgets([...widgets, widgetType]);
  }

  function handleDragOver(e){
    e.preventDefault();
  }

  function handleTrashOnDrag(e, index) {
    e.dataTransfer.setData("currentIndex", index);
  }

  function handleTrashOnDrop(e) {
    console.log("current index:", e.dataTransfer.getData("currentIndex"));
    removeWidget(parseInt(e.dataTransfer.getData("currentIndex")));
  }

  useEffect(() => {
    fetch("/api").then(
      response => response.json()
    ).then(
      data => {
        setBackendData(data)
      }
    )
  }, []);

  return (
    <div className="App">
      <div className="widgets">
        <div 
          className="widget"
          draggable
          onDragStart={(e) => handleOnDrag(e, "Widget A")}
        >
          Widget A
        </div>
        <div 
          className="widget"
          draggable
          onDragStart={(e) => handleOnDrag(e, "Widget B")}
        >
          Widget B
        </div>
        <div 
          className="widget"
          draggable
          onDragStart={(e) => handleOnDrag(e, "Widget C")}
        >
          Widget C
        </div>
        <div className="page" onDrop={handleOnDrop} onDragOver={handleDragOver}>
          {widgets.map((widget, index) => (
            <div className="droppedWidget" draggable key={index} onDragStart={(e)=> handleTrashOnDrag(e, index)}>
              {widget}
            </div>
          ))}
        </div>
        <div className="trash" onDrop={handleTrashOnDrop} onDragOver={handleDragOver}>
          Trash Bin
        </div>
      </div>
    </div>
    // <div>
    //   {(typeof backendData.users === 'undefined') ? (
    //     <p>Loading...</p>
    //   ): (
    //     backendData.users.map((user, i) => (
    //       <p key={i}>{user}</p>
    //     ))
    //   )}
    // </div>
  );
}

export default App