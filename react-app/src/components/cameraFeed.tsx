import React, { useEffect, useState } from 'react';

export default function CameraData({
    ...props
}) {

    const [imageData, setData] = useState("");
    const [widgetID, setID] = useState(Math.floor(Math.random()*50000));

    useEffect(() => {
        

        return(() => {

        })
    }, [imageData]);

    return (
        <>
            <img src={`data:image/png;base64,${imageData}`}></img>
            <input id={`${widgetID}`} value=""></input>
            <button onClick={() => {socket.emit('image_subscription', document.getElementById(`${widgetID}`).value)}}>Submit</button>
        </>
    );
}