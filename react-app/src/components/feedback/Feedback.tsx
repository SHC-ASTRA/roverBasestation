import React, {useState, useEffect, useRef, MutableRefObject} from 'react'
import './Feedback.css'

export const Feedback = ({
    topicName = '/core/feedback'
    }) => {
    const [data, setData] = useState([]);
    const textRef: MutableRefObject<HTMLTextAreaElement | null> = useRef(null);
    const checkboxRef: MutableRefObject<HTMLInputElement | null> = useRef(null);
    // Widget container
    // var widgetContainer;
    var innerText;

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((response) => {setData(response['data']);})
        }, 1000);

        // If the TextRef exists for an element and the user has autoscroll enabled
        // Scroll down the text element to the bottom 
        if(textRef.current && checkboxRef.current && checkboxRef.current.checked) {
            textRef.current.scrollTo(0, textRef.current.scrollHeight);
        }
        
        return(() => {
            clearInterval(intervalValue);
        })
    }, [data]);

    if(typeof data == "object" && data.length && data.length > 1) {
        innerText = data.join('\r\n');
    } else {
        innerText = data;
    }

    // Handle looping and grabbing the parent of the current assignment
    // until the widget container is found
    /* if(textRef.current) {
            while( !widgetContainer || (widgetContainer.classList && Array.from(widgetContainer.classList).includes("widget-content")) ) {
            widgetContainer = textRef.current.parentElement;
        }
    } */

    // Return the textbox
    return (
        <>
            <div>
                <label>
                    AutoScroll:&nbsp;
                    <input ref={checkboxRef} type="checkbox" defaultChecked></input>
                </label>
            </div>
            <div>
                <textarea value={innerText} disabled ref={textRef} rows={6} wrap="off" ></textarea>
            </div>
        </>
    )
}