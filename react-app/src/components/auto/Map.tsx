import React, {useState, useEffect} from 'react'

export const Map = ({
    topicName = '/astra/auto/statusUpdate'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/auto/statusUpdate')
                .then((response) => response.json())
                .then((data) => setData(data[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <>
        
        </>
    )
}