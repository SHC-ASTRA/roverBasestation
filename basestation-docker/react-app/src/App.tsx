import React, { useState, useEffect } from 'react'; 
import { SimpleGamepadDisplay } from './widgets/simpleGamepad.tsx';

function Header() {
  return (
    <></>
  )

}

function ProgressBar() {
  const [progress_value, setProgress] = useState(0); 
  
    useEffect(() => { 

        //Implementing the setInterval method 
        const interval = setInterval(() => { 
            if(progress_value >= 100) {
                setProgress(0);
            } else {
                setProgress(progress_value + 0.5);
            }
        }, 250); 
    
        //Clearing the interval 
        return () => clearInterval(interval); 
    }, [progress_value]); 
    return (
        <div
            className="progress"
            role="progressbar"
            aria-label="Example with label"
            aria-valuenow={progress_value}
            aria-valuemin={0}
            aria-valuemax={100}
        >
            <div className="progress-bar" style={{ width: `${progress_value}%` }}>
                {progress_value}%
            </div>
        </div>
    )
}

function App() {
  // useEffect(() => {
  //   fetch("/api").then(
  //     response => response.json()
  //   ).then(
  //     data => {
  //       setBackendData(data)
  //     }
  //   )
  // }, []);
  // [] is the list of dependencies

  return (
    <div>
      <Header />
      <SimpleGamepadDisplay />
    </div>
  )
}

export default App