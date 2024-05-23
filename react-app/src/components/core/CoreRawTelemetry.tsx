import React, {useState, useEffect} from 'react'

type Vec3D = {
    x: number,
    y: number,
    z: number
}

type GPS = {
    lat: number,
    long: number
}

export const Telemetry = ({
    topicName='/core/telemetry'
}) => {
    const [accel, setAccel] = useState<Vec3D>();
    const [altitude, setAlt] = useState<number>();
    const [gpsCoords, setGPSCoords] = useState<GPS>();
    const [gpsSats, setGPSSats] = useState<number>();
    const [gyro, setGyro] = useState<Vec3D>();
    const [orientation, setOrientation] = useState<number>();
    const [pressure, setPressure] = useState<number>();
    const [temperature, setTemperature] = useState<number>();

    useEffect(() => {
        let intervalValue = setInterval(() => {
            fetch(topicName)
                .then((response) => response.json())
                .then((data) => {
                    if (data['acc_x'] && data['acc_y'] && data['acc_z']) setAccel({x: data['acc_x'], y: data['acc_y'], z: data['acc_z']});
                    if (data['alt']) setAlt(data['alt']);
                    if (data['gps_lat'] && data['gps_long']) setGPSCoords({lat: data['gps_lat'], long: data['gps_long']});
                    if (data['gps_sat']) setGPSSats(data['gps_sat']);
                    if (data['gyro_x'] && data['gyro_y'] && data['gyro_z']) setGyro({x: data['gyro_x'], y: data['gyro_y'], z: data['gyro_z']});
                    if (data['orient']) setOrientation(data['orient']);
                    if (data['pres']) setPressure(data['pres']);
                    if (data['temp']) setTemperature(data['temp']);
                })
        }, 2000);

        return(() => {
            clearInterval(intervalValue);
        })
    }, []);

    return (
        <>
            <div>{`GPS Latitude: ${gpsCoords ? gpsCoords.lat : ""}, Longitude: ${gpsCoords ? gpsCoords.long : ""}`}</div>
            <div>{`GPS Satellites: ${gpsSats ? gpsSats : ""}`}</div>
            <div>{`Gyroscope: ${gyro ? gyro.x : ""}, ${gyro ? gyro.y : ""}, ${gyro ? gyro.z : ""}`}</div>
            <div>{`Acceleration: ${accel ? accel.x : ""}, ${accel ? accel.y : ""}, ${accel ? accel.z : ""}`}</div>
            <div>{`Orientation: ${orientation ? orientation : ""}`}</div>
            <div>{`Temperature: ${temperature ? temperature : ""}`}</div>
            <div>{`Altitude: ${altitude ? altitude : ""}`}</div>
            <div>{`Pressure: ${pressure ? pressure : ""}`}</div>
        </>
    )
}