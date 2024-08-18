import React, { useEffect, useState } from 'react';

export default function LiveData({
    topicName = "/astra/core/feedback", // String socketio event
    ...props
}) {

    const [data, setData] = useState([]);

    useEffect(() => {
        // TODO: Reimplement using fetch handling to the backend api
        let intervalValue = setInterval(() => {
            fetch('/message_data')
                .then((response) => response.json())
                .then((data) => setData(data[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <>
            {data}
        </>
    );
}