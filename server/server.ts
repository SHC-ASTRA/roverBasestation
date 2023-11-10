/* Base Imports */
///////////////////////////////////////
import * as rclnodejs from 'rclnodejs';
//import * as express from 'express';
import {Server, IncomingMessage, ServerResponse} from 'http';
import * as http from 'http';
import { readFile } from 'fs';
/* Custom Imports */
///////////////////////////////////////
import {RosNode} from './ros'

// const app = express();
const port: number = Number(process.env.PORT) || 8000;

rclnodejs.init().then(() => {
    const node = new RosNode('/basestation/PUBLISHER');
    // Regularly publish data
    setInterval(
        // Anonymous callback
        () => {
            node.publishData("Hello ROS 2 from rclnodejs"); // Publish data
        }, 1000);
    // Spin the node
    node.spin();
});

//app.listen(port, () => {console.log(`Base Station Server started on port ${port}`)});

var server:Server = http.createServer(
    // HTTP server request callback
    (req: IncomingMessage, res: ServerResponse) => {
	res.writeHead(200, { 'Content-Type': 'text/plain' });
	res.end('okay')
    return
    // Never reached
    readFile(__dirname + req.url, (err, data) => {
        if (err) {
          res.writeHead(404, { 'Content-Type': 'text/html' });
          res.end('404: File not found');
        } else {
          res.writeHead(200, { 'Content-Type': 'text/html' });
          res.end(data);
        }
      });
});

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON ${port}`);
});
