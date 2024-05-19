import React, { useEffect, useState } from 'react';

export default function CameraData({
    defaultTopic,
    defaultScale,
    defaultRotation,
    ...props
}) {

    const [topic, setTopic] = useState(defaultTopic || "");
    const [imageData, setData] = useState({data:undefined});
    const [scale, setScale] = useState(defaultScale || 90);
    const [rotation, setRotation] = useState(defaultRotation || 0)

    useEffect(() => {
        socket.on(topicName, (image) => {
            setData(image);
        })
    }, [imageData])

    return (
        <>
            <div class="dropdown">
                <button class="btn btn-secondary dropdown-toggle" type="button" id="dropdownMenuButton" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
                    Dropdown button
                </button>
                <div class="dropdown-menu" aria-labelledby="dropdownMenuButton">
                    <a class="dropdown-item" href="#">Action</a>
                    <a class="dropdown-item" href="#">Another action</a>
                    <a class="dropdown-item" href="#">Something else here</a>
                </div>
            </div>
            <img width={`${scale}%`} height={`${scale}%`} style={{transform: `rotate(${rotation}deg)`}} src={`data:image/png;base64,${imageData.data}`}></img>
            <div>
                Scale: 
                <input type='number' value={scale} 
                    onChange={ (event) => {
                        let val = Number(event.target.value)
                        // Clamp value
                        if(val < 10 || val > 200) {
                            if(val < 10) val = 10;
                            if(val > 200) val = 200;
                        }
                        setScale(val)
                    }
                }>
                </input>
            </div>
            <div>
                <button onClick={() => setRotation(rotation + 90)}>
                    Left
                </button>
                <button onClick={() => setRotation(rotation - 90)}>
                    Right
                </button>
                <button onClick={() => setRotation(rotation + 180)}>
                    Horizontal Flip
                </button>
            </div>
        </>
    );
}