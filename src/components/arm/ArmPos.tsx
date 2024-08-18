import React, {useState, useEffect} from 'react'

export const ArmPos = ({
    topicName = '/arm/feedback'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/arm/feedback')
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