import React, { useEffect, useState } from 'react';

export default function LiveData({
    eventName = "/feedback/livedata", // String socketio event
    children,
    ...props
}) {

    const [data, setData] = useState("");

    useEffect(() => {

        // Socket events
        socket.on(eventName, (data) => {
            setData(data);
            console.log(data);
        });
    });

    return (
        <>
            {data}
            {children}
        </>
    );
}