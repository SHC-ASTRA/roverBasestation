import React, {useState, useEffect} from 'react';
import "./ConnectionLatency.css"

export const ConnectionStatus = ({
    topicName = "/core/ping"
}) => {
    const [connectionStatus, setConnectionStatus] = useState<number>(0);

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => {
                    if (data['data'] == "") setConnectionStatus(0);
                    else setConnectionStatus(Math.round(data['data'] * 1000000) / 1000);
            })
        }, 5000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, []);


    return (
        <div style={{display:'flex', flexDirection:'row'}}>
            <div className="dot" id={connectionStatus === 0 ? "gray" : connectionStatus < 250 ? "green" : connectionStatus < 350 ? "yellow" : "red"}> </div>
            <div className="connection-status">{connectionStatus ? "Connected: " + connectionStatus + " ms": "Failed to connect."}</div>
        </div>
    )
}