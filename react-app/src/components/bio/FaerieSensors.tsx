import React, {useState, useEffect} from 'react'

export const FaerieSensors = ({
    topicName = '/astra/bio/feedback'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/bio/feedback')
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