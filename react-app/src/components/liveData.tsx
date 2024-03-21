import React, { useEffect, useState } from 'react';

export default function LiveData({
    eventName = "/feedback/livedata", // String socketio event
    children,
    ...props
}) {

    const [data, setData] = useState("");

    useEffect(() => {
        // TODO: Reimplement using fetch handling to the backend api
    });

    return (
        <>
            {data}
            {children}
        </>
    );
}