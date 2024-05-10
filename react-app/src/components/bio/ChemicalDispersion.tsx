import React, {useState, useEffect} from 'react'

export const ChemicalDispersion = ({
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

    const setDispersion = () => setData({});

    return (
        <div> 
            <div className="button-wrapper">
                <button className="round-button" onClick={setDispersion}>
                    1
                </button>
                <button className="round-button" onClick={setDispersion}>
                    2
                </button>
                <button className="round-button" onClick={setDispersion}>
                    3
                </button>
                <button className="round-button" onClick={setDispersion}>
                    4
                </button>
            </div>
            <div className="button-wrapper">
                <button className="red-button" onClick={setDispersion}> </button>
                <button className="red-button" onClick={setDispersion}> </button>
                <button className="red-button" onClick={setDispersion}> </button>
                <button className="red-button" onClick={setDispersion}> </button>
            </div>
        </div>
    )
}