import React, { useEffect, useState } from 'react';

export default function CameraData({
    topicName,
    ...props
}) {

    const [imageData, setData] = useState({data:undefined});

    useEffect(() => {
        socket.on(topicName, (image) => {
            setData(image);
        })
    }, [imageData])

    return (
        <>
            <img src={`data:image/png;base64,${imageData.data}`}></img>
            
        </>
    );
}