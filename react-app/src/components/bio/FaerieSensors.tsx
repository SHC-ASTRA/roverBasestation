import React, {useState, useEffect} from 'react'

type BioResponse = {
    humidity: Number,
    temperature: Number
}

export const FaerieSensors = ({
    topicName = '/bio/feedback'
    }) => {
    const [data, setData] = useState<BioResponse>({humidity: 0, temperature: 0});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => setData(data[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <>
            <p>Humidity: {data.humidity}<br />
            Temperature: {data.temperature}</p>
        </>
    )
}