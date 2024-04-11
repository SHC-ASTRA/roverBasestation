import React, {useState, useEffect} from 'react'

export const CoreControl = ({
    topicName = '/astra/core/control'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/core/control')
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