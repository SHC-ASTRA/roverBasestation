import React, {useState, useEffect} from 'react'

export const ArmLaser = ({
    topicName = '/astra/arm/control'
    }) => {
    const [data, setData] = useState({});

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch('/arm/control')
                .then((response) => response.json())
                .then((data) => setData(data[topicName]))
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    const toggleLaser = () => setData({});

    return (
        <div>
            <button className="laser-button" onClick={toggleLaser}>
                Laser
            </button>
        </div>
    )
}