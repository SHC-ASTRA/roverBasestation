import React, {useState, useEffect} from 'react'

export const FanControl = ({
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

    const setFans = () => {
        setData({})
        console.log("click!")
    };

    return (
        <div>
            <div className="button-wrapper">
            <button className="round-button" onClick={setFans}>
                Fan 1
            </button>
            <button className="round-button" onClick={setFans}>
                Fan 2
            </button>
            <button className="round-button" onClick={setFans}>
                Fan 3
            </button>
            <button className="round-button" onClick={setFans}>
                Fan 4
            </button>
            </div>
        </div>
    )
}