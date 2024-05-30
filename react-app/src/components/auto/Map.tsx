import React, {useState, useEffect} from 'react'
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet'
import "./Map.css"
import { LatLngExpression, Icon } from "leaflet"

export const Map = ({
    topicName='/core/telemetry'
    }) => {

    const [gpsCoords, setGPSCoords] = useState({
        longitude: 0,
        latitude: 0
    });

    const [gpsPoints, setGPSPoints] = useState<LatLngExpression[]>([]);
   
    const cluckyIcon = new Icon({
        iconUrl : "clucky.png",
        iconSize : [42, 40]
    });
    

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => {
                    if (data['gps_long'] && data['gps_lat']) {
                        const newPoint = { latitude: data['gps_lat'], longitude: data['gps_long'] };
                        setGPSCoords(newPoint);
                        setGPSPoints(prevPoints => [...prevPoints, [newPoint.latitude, newPoint.longitude]]);
                    }    
                })
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, []);

    console.log(gpsPoints);

    function ChangeView({ center }) {
        const map = useMap();
        map.setView(center)

        return null;
    }

    return (
        <div >
            <div className='leaflet-container'>
                <MapContainer center={[34.727, -86.639]} zoom={10} scrollWheelZoom={true} id="map" >
                    <ChangeView center={[gpsCoords.latitude, gpsCoords.longitude]} />
                    <TileLayer
                        url="./tiles/{z}/{x}/{y}.png"
                    />
                    <Marker position={[gpsCoords.latitude, gpsCoords.longitude]} icon={cluckyIcon}> </Marker>
                    <Polyline positions={gpsPoints} color="red"/>
                </MapContainer>
            </div>
            
            Current position: [ {gpsCoords.latitude} , {gpsCoords.longitude} ]
        </div>
    )
}