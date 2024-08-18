import React, {useState, useEffect} from 'react'
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet'
import "./Map.css"
import { LatLngExpression, Icon } from "leaflet"

type Waypoint = {
    coords: LatLngExpression
    annotation?: string
}

export const Map = ({
    topicName='/core/telemetry'
    }) => {

    const [gpsCoords, setGPSCoords] = useState({
        longitude: 0,
        latitude: 0
    });

    const [gpsPoints, setGPSPoints] = useState<LatLngExpression[]>([]);
    const [pointsLength, setLength] = useState<number>(0);

    const [latitude, setLat] = useState<number>(0);
    const [longitude, setLong] = useState<number>(0);
    const [annotation, setAnnotation] = useState<string | undefined>();
    const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
   
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
                        if (pointsLength == 0
                            || gpsPoints[pointsLength - 1][0] != data['gps_lat'] 
                            || gpsPoints[pointsLength - 1][1] != data['gps_long']) {
                            
                            console.log(pointsLength);
                            if (pointsLength != 0) {
                                console.log(gpsPoints[pointsLength - 1][0], data['gps_lat'])
                                console.log(gpsPoints[pointsLength - 1][1], data['gps_long'])
                            }
                            setLength(previousLength => previousLength + 1);
                            setGPSPoints(prevPoints => [...prevPoints, [newPoint.latitude, newPoint.longitude]]);
                        }
                    }    
                })
        }, 1000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, []);

    function ChangeView({ center }) {
        const map = useMap();
        map.setView(center)

        return null;
    }

    return (
        <div >
            <div className='leaflet-container'>
                <MapContainer center={[38.39621, -110.79357]} zoom={10} scrollWheelZoom={true} id="map" >
                    <ChangeView center={[gpsCoords.latitude, gpsCoords.longitude]} />
                    <TileLayer
                        url="./tiles/{z}/{x}/{y}.png"
                    />
                    {waypoints.map((site) => {
                        console.log(site.coords);
                        return (<Marker position={site.coords}>
                            <Popup>
                                <div>{`${site.annotation ? site.annotation : ""}\n
                                Latitude: ${site.coords[0]}, Longitude: ${site.coords[1]}`}</div>
                                
                            </Popup>
                        </Marker>)
                    })}
                    <Marker position={[gpsCoords.latitude, gpsCoords.longitude]} icon={cluckyIcon}> </Marker>
                    <Polyline positions={gpsPoints} color="red"/>
                </MapContainer>
            </div>
            
            <div>Current position: [ {gpsCoords.latitude} , {gpsCoords.longitude} ]</div>

            <div style={{display: 'flex', flexDirection: 'row', alignItems: "center", justifyContent: "space-evenly"}}>
                <input className="text-input" type="text" placeholder="GPS Latitude" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 180 || value < -180) return;
                    setLat(value);
                }}></input>
                <input className="text-input" type="text" placeholder="GPS Longitude" onChange={(e) => {
                    let value: number = Number(e.target.value);
                    if (Number.isNaN(value) || value > 180 || value < -180) return;
                    setLong(value);
                }}></input>
                <input className="text-input" type="text" placeholder="Waypoint Annotation" onChange={(e) => {
                    let value: string = e.target.value;
                    setAnnotation(value);
                }}></input>

                <button  className="control-button" onClick={() => {
                    if (latitude === 0 || longitude === 0) return;
                    let waypoint: Waypoint = {
                        coords: [latitude, longitude],
                        annotation: annotation
                    }
                    setWaypoints(previousWaypoints => [...previousWaypoints, waypoint]);
                    console.log(`Adding waypoint\nLatitude: ${waypoint.coords[0]}, Longitude: ${waypoint.coords[1]}`)
                }}>
                    Add Waypoint
                </button>
            </div>
        </div>
    )
}