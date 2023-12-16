/* Base Imports */
import * as path from 'path'
/* RCLNodejs */
import * as rclnodejs from 'rclnodejs';
/* Custom ROS2 Node Implementation */
import {RosNode} from './ros'
/* HTTP */
import * as http from 'http';
import {Server as HttpServer, IncomingMessage, ServerResponse} from 'http';
/* Express */
import * as express from 'express';
const app = express();
/* Socket.io */ 
import { Server as SocketServer } from "socket.io";

// const app = express();
const port: number = Number(process.env.PORT) || 8000;

rclnodejs.init().then(() => {
    // Create the node
    const node = new RosNode();
    node.createTopic('astra/core/control');
    /* setInterval(
        () => node.publishData('astra/core/control', 'Hello from the Basestation!'),
        10000        
    ); */
    node.spin();
});

// Host the build react page root
app.use(
    '/',
    express.static(path.join(__dirname, "..", "react-app", "build"))
);

// Implement Application/JSON POST request handling middleware
// for all /api/ paths
app.use(
    '/api/*',
    express.json()
);

// Handle GET requests that provide an ID path
app.get(
    '/api/:id', (req, res) => {
        return res.send(`API in Progress. Received ID ${req.params.id}`);
    }
);

// Gamepad status
{/* 
{
    buttons: {
        a: bool,
        b: bool,
        x: bool,
        y: bool,
        left_bumper: bool,
        right_bumper: bool,
        left_trigger: bool,
        right_trigger: bool,
        view: bool,
        menu: bool,
        d_up: bool,
        d_down: bool,
        d_left: bool,
        d_right: bool
    },
    trigger_vals: {
        left_trigger_val: float,
        right_trigger_val: float
    },
    stick_vals: {
        left_hval: float,
        left_vval: float,
        right_hval: float,
        right_vval: float
    }
}
*/}

// Arm control API endpoint
app.post(
    '/api/arm/control', (req, res) => {
        // Handle ROS messaging
        return res.send('Recieved.');
    }
)

// POST debug endpoint
app.post(
    '/api/debug', (req, res) => {
        console.log(JSON.stringify(req.body));
        // Return the body
        return res.send(req.body);
    }
)

// HTTP server makes use of the Express handler
var server: HttpServer = http.createServer(app);

// Create an socket connection to the HTTP server session
const io = new SocketServer(server);

// Socket connection handlers
io.on('connection', (socket) => {
    console.log('Websocket Connection');
    
    // Disconnection event handler
    socket.on('disconnect', () => {
      console.log('Websocket Disconnection.');
    });

    /* Other Event Handlers */

    // Debug
    socket.on('debug', (data) => {
        console.log(`Websocket Debug: ${data}`);
    });
});

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON ${port}`);
});
