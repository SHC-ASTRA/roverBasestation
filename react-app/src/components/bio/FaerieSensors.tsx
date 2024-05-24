import React, {useState, useEffect} from 'react'

type BioResponse = {
    humidity: Number,
    temperature: Number
}

export const FaerieSensors = ({
    topicName = '/arm/bio/feedback'
    }) => {
    const [data, setData] = useState<BioResponse>({humidity: 0, temperature: 0});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => setData({humidity: data.humidity, temperature: data.temperature}))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <>
            <p>Humidity: {data.humidity ? data.humidity : "No data received."}<br />
            Temperature: {data.temperature ? data.temperature : "No data received."}</p>
        </>
    )
}