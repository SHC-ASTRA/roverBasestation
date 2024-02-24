import React, { useEffect, useState } from 'react';

export default function VisualGamepad({
    eventName = "/feedback/livedata",
    ...props
}) {

    const [data, setData] = useState("");

    useEffect(() => {

        // Socket events
        socket.on(eventName, (data) => {
            setData(data);
        });
    });

    return (
        <div>

        </div>
    );
}
