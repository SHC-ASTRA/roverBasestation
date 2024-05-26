import React, {useState, useEffect} from 'react'
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet'
import "./Map.css"
import L, { LatLngExpression, Icon } from "leaflet"

export const Map = ({
    topicName = '/astra/auto/feedback'
    }) => {

    const [gpsCoords, setGPSCoords] = useState({
        longitude: 0,
        latitude: 0
    })

    let points: LatLngExpression[] = [
        [38.40643400834641, -110.79181336126874]
    ]

    let lastPoint = points[points.length - 1]
   
    const cluckyIcon = new Icon({
        iconUrl : "clucky.png",
        iconSize : [42, 40]
    })
    

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
            fetch('/auto/feedback')
                .then((response) => response.json())
                .then((data) => {
                    setGPSCoords({longitude: data['gps_long'], latitude: data['gps_lat']})
                })
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
            // points.push([data.latitude, data.longitude])
        })
    }, [data]);

    function ChangeView({ center, zoom }) {
        const map = useMap();
        map.setView(center, zoom)

        return null;
    }

    const [state, setState] = useState({
        longitude: 0,
        latitude: 0,
    });

    // user location
    useEffect(() => {
        navigator.geolocation.getCurrentPosition(
            function (position) {
                console.log(position);
                setState({
                    longitude: position.coords.longitude,
                    latitude: position.coords.latitude,
                });
            },
            function (error) {
                console.error("Error: " + error.code + " - " + error.message);
            },
            {
                enableHighAccuracy: true,
            });
        }, 
    []);

    let points: LatLngExpression[] = [
        [38.40643400834641, -110.79181336126874]
    ]

   let lastPoint = points[points.length - 1]
   
    for (let i=0; i < 15; i++) {
        lastPoint = points[points.length - 1]
        points.push([lastPoint[0] - 0.0002 * Math.random(), lastPoint[1] + 0.0005 * Math.random()])
    }

    const cluckyIcon = new Icon({
        iconUrl : "clucky.png",
        iconSize : [42, 40]
    })
    

    console.log(points)
    // "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"

    return (
        <div >
            <div className='leaflet-container'>
                <MapContainer center={[34.727, -86.639]} zoom={10} scrollWheelZoom={true} id="map" >
                    <ChangeView center={lastPoint} zoom={15} />
                    <TileLayer
                        url="./tiles/{z}/{x}/{y}.png"
                    />
                    <Marker position={[state.latitude, state.longitude]}>
                        <Popup>
                            User location.
                        </Popup>
                    </Marker>
                    <Marker position={points[points.length - 1]} icon={cluckyIcon}> </Marker>
                    <Polyline positions={points} color="red"/>
                </MapContainer>
            </div>
            
            Current position: [ {lastPoint[0]} , {lastPoint[1]} ]
            <div>
                Orientation: 
            </div>
        </div>
    )
}