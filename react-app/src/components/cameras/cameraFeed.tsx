import React, { useEffect, useState } from 'react';

export default function CameraData({
    defaultTopic="",
    defaultScale=90,
    defaultRotation=0,
    ...props
}) {
    // Camera Feed Topic Name
    const [topicName, setTopic] = useState(defaultTopic || "");
    // Camera Image Data
    const [hasRequested, setRequested] = useState(false);
    const [imageData, setData] = useState({data:undefined});
    // Camera Feed Settings
    const [scale, setScale] = useState(defaultScale || 90);
    const [rotation, setRotation] = useState(defaultRotation || 0);
    // Available Camera Feed Topics
    const [availableTopics, setAvailableTopics] = useState([<></>]);

    useEffect(() => {
        // Initalize the socket's image subscriber, if it has not already
        if(!hasRequested)
        {
            socket.emit('image_subscription', topicName);
            setRequested(true);
        }

        console.log("Attempting to fetch /api/camera_topics")
        fetch('/api/camera_topics')
            .then((response) => response.json())
            .then((topicObject) => {
                let temp_arr: JSX.Element[] = []
                // Clear available topics
                // This is intended Javascript usage of the "in" keyword
                // as the keys are enumerated in this use case
                // https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Statements/for...in
                for(let topicKey in topicObject) {
                    // topicKey is currently the topic name, but is named differently to prevent
                    // confusion in regards to the scope of the state variable

                    // Update a temporary array, you cannot set a state variable rapidly
                    // within a for-loop because state setting being asynchronous
                    temp_arr.push(
                        <a className='dropdown-item' href='#' onClick={
                            () => {
                                // Update the back-end subscribers
                                socket.emit('connection_change', {topicName}, {topicKey});
                                // Reset the variable to make the camera re-request a camera subscriber
                                setRequested(false);
                                // Set the new topic inside the pertaining state variable
                                setTopic({topicKey});
                            }
                        }>{topicKey}</a>
                    );
                }
                // Assign the array of elements to the available topics
                setAvailableTopics(temp_arr);
            });

        // Request data for dropdown
        let intervalValue = setInterval(() => {
            
        }, 1000);

        // Create event listener that can be deleted later
        let socketEventListener = socket.on(topicName, (image) => {
            setData(image);
        });

        // Deconstructor
        // Expected to be called when the state variables change
            return () => {
            // Remove the socket listener so there are not multiple listeners
            // trying to modify the image at the same time
            
            // This will be called and update the listener each time
            // the image data is updated by the listener 
            socket.off(topicName, socketEventListener);

            // Clear timeout
            clearInterval(intervalValue);
        }
    }, [imageData])

    return (
        <>
            <div class="dropdown">
                <button class="btn btn-secondary dropdown-toggle" type="button" id="dropdownMenuButton" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
                    Dropdown button
                </button>
                <div class="dropdown-menu" aria-labelledby="dropdownMenuButton">
                    {availableTopics}
                    {/* <a class="dropdown-item" href='#'>{availableTopics}</a>
                    <a class="dropdown-item" href="#">Action</a>
                    <a class="dropdown-item" href="#">Another action</a>
                    <a class="dropdown-item" href="#">Something else here</a> */}
                </div>
            </div>
            <img width={`${scale}%`} height={`${scale}%`} style={{transform: `rotate(${rotation}deg)`}} src={`data:image/png;base64,${imageData.data}`}></img>
            <div>
                Scale: 
                <input type='number' value={scale} 
                    onChange={ (event) => {
                        let val = Number(event.target.value)
                        // Clamp value
                        if(val < 10 || val > 200) {
                            if(val < 10) val = 10;
                            if(val > 200) val = 200;
                        }
                        setScale(val)
                    }
                }>
                </input>
            </div>
            <div>
                <button onClick={() => setRotation(rotation + 90)}>
                    Left
                </button>
                <button onClick={() => setRotation(rotation - 90)}>
                    Right
                </button>
                <button onClick={() => setRotation(rotation + 180)}>
                    Horizontal Flip
                </button>
            </div>
        </>
    );
}