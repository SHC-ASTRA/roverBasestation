import React, {useState, useEffect} from 'react'
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet'
import "./Map.css"
import L, { LatLngExpression, Icon } from "leaflet"

export const Map = ({
    topicName='/core/telemetry'
    }) => {

    const [gpsCoords, setGPSCoords] = useState({
        longitude: 0,
        latitude: 0
    })

    let points: LatLngExpression[] = [
        [38.40643400834641, -110.79181336126874]
    ]
   
    const cluckyIcon = new Icon({
        iconUrl : "clucky.png",
        iconSize : [42, 40]
    })
    

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => {
                    if (data['gps_long'] && data['gps_lat']) {
                        setGPSCoords({longitude: data['gps_long'], latitude: data['gps_lat']});
                        if (points[points.length - 1][0] != gpsCoords.latitude || points[points.length - 1][1] != gpsCoords.longitude) {
                            points.push([gpsCoords.latitude, gpsCoords.longitude]);
                        }
                    }    
                })
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, []);

    function ChangeView({ center, zoom }) {
        const map = useMap();
        map.setView(center, zoom)

        return null;
    }

    // user location
    useEffect(() => {
        navigator.geolocation.getCurrentPosition(
            function (position) {
                console.log(position);
                setGPSCoords({
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

    console.log(points)
    // "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"

    return (
        <div >
            <div className='leaflet-container'>
                <MapContainer center={[34.727, -86.639]} zoom={10} scrollWheelZoom={true} id="map" >
                    <ChangeView center={points[points.length - 1]} zoom={15} />
                    <TileLayer
                        url="./tiles/{z}/{x}/{y}.png"
                    />
                    <Marker position={[gpsCoords.latitude, gpsCoords.longitude]}>
                        <Popup>
                            User location.
                        </Popup>
                    </Marker>
                    <Marker position={points[points.length - 1]} icon={cluckyIcon}> </Marker>
                    <Polyline positions={points} color="red"/>
                </MapContainer>
            </div>
            
            Current position: [ {points[points.length - 1][0]} , {points[points.length - 1][1]} ]
        </div>
    )
}