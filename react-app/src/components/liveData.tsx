import React, { useEffect, useState } from 'react';

export default function LiveData({
    topicName = "/astra/core/feedback", // String socketio event
    children,
    ...props
}) {

    const [data, setData] = useState([]);

    useEffect(() => {
        // TODO: Reimplement using fetch handling to the backend api
        let intervalValue = setInterval(() => {
            fetch('/message_data')
                .then((response) => response.json())
                .then((responseData) => setData(responseData[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    let listElements = [];

    data.forEach((message, index) => {
        listElements.push(<li key={index}>{message}</li>);
      });

    return (
        <>
            <div>{data}</div>
            {children}
        </>
    );
}