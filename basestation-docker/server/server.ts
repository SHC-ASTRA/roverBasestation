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

// The ROSNode
var node;

rclnodejs.init().then(() => {
    // Create the node
    node = new RosNode();
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

    // Basic Controller Event Handler, TEST
    // Left X, Left Y, Right X, Right Y
    socket.on('axes', (lx, ly, rx, ry) => {
        console.log(`Axes Event: ${lx}, ${ly}, ${rx}, ${ry}`);
    })

    socket.on('/core/control', (lx, ly, rx, ry) => {
        // If they are all zeroed out
        // the controller is disconnected or the page is unfocused
        if((lx + ly + rx + ry) == 0) return
        console.log(`/core/control Event: ${lx}, ${ly}, ${rx}, ${ry}`);
        node.publishData('astra/core/control', `ctrl,${lx.toFixed(4)},${ly.toFixed(4)},${rx.toFixed(4)},${ry.toFixed(4)}`)
    })
});

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON ${port}`);
});
