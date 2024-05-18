import React, {useState, useEffect} from 'react'

export const CITADELFeedback = ({
    topicName = '/bio/feedback'
    }) => {
    const [data, setData] = useState([]);

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => setData(data['data']))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <>
            <div>{data ? data[data.length - 1] : "No data received."}</div>
        </>
    )
}