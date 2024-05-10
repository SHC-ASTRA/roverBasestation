import React, {useState, useEffect} from 'react'

export const FaerieLaser = ({
    topicName = '/astra/bio/control'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/bio/control')
                .then((response) => response.json())
                .then((data) => setData(data[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    return (
        <div> 
        <button className="laser-button" >
            Laser
        </button>
        </div>
    )
}