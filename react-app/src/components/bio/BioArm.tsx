import React, {useState, useEffect} from 'react'

export const BioArm = ({
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

    const setHome = () => setData({});

    const setPositions = () => setData({});

    return (
        <div className="button-wrapper">
            <button className="control-button" onClick={setHome}>
                Homing
            </button>
            <button className="control-button" onClick={setPositions}>
                Stow
            </button>
            <button className="control-button" onClick={setPositions}>
                Zero
            </button>
            <button className="control-button" onClick={setPositions}>
                Extend
            </button>
            <button className="control-button" onClick={setPositions}>
                Max
            </button>
        </div>
    )
}