/* Base Imports */
import * as path from 'path'
/* Application Specifci Imports */
import * as rclnodejs from 'rclnodejs';
// HTTP
import * as http from 'http';
import {Server, IncomingMessage, ServerResponse} from 'http';
// Express
import * as express from 'express';
const app = express();
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

app.use(
    express.static(path.join(__dirname, "..", "react-app", "build"))
  );

var server:Server = http.createServer(app);

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON ${port}`);
});
