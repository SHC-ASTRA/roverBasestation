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
    const [accel, setAccel] = useState<Vec3D>({x: 0, y: 0, z: 0});
    const [altitude, setAlt] = useState<number>(0);
    const [gpsCoords, setGPSCoords] = useState<GPS>({lat: 0, long: 0});
    const [gpsSats, setGPSSats] = useState<number>(0);
    const [gyro, setGyro] = useState<Vec3D>({x: 0, y: 0, z: 0});
    const [orientation, setOrientation] = useState<number>(0);
    const [pressure, setPressure] = useState<number>(0);
    const [temperature, setTemperature] = useState<number>(0);

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
            <div>{`GPS Latitude: ${gpsCoords.lat}, Longitude: ${gpsCoords.long} degrees`}</div>
            <div>{`GPS Satellites: ${gpsSats}`}</div>
            <div>{`Gyroscope: ${gyro.x}, ${gyro.y}, ${gyro.z} rad/s`}</div>
            <div>{`Acceleration: ${accel.x}, ${accel.y}, ${accel.z} m/s^2`}</div>
            <div>{`Orientation: ${orientation} degrees`}</div>
            <div>{`Temperature: ${temperature} deg C`}</div>
            <div>{`Altitude: ${altitude} m`}</div>
            <div>{`Pressure: ${pressure} Pa`}</div>
        </>
    )
}