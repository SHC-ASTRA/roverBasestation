import React, {useState, useEffect} from 'react'

export const CoreFeedback = ({
    topicName = '/astra/core/feedback'
    }) => {
    const [data, setData] = useState([]);

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/core/feedback')
                .then((response) => response.json())
                .then((data) => {setData(data['data'])})
        }, 1000);


        return(() => {
            clearInterval(intervalValue);
        })
    }, []);

    return (
        <>
            <div>{data ? data[data.length - 1] : "No data received."}</div>
        </>
    )
}