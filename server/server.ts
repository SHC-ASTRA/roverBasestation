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
import { Socket } from 'dgram';

// const app = express();
const port: number = Number(process.env.PORT) || 8000;

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

// Open sockets
let openSockets = [];

// Socket connection handlers
io.on('connection', (socket) => {
    console.log('Websocket Connection');
    openSockets.push(socket);
    
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

    socket.on('/core/control', (ly, ry) => {
        // If they are all zeroed out
        // the controller is disconnected or the page is unfocused
        // if((ly + ry) == 0) return
        console.log(`/core/control Event: ${ly}, ${ry}`);
        node.publishData('astra/core/control', `ctrl,${ly.toFixed(4)},${ry.toFixed(4)}`)
    })
});

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
    
    node.initalizeHealthPackets((request, response) => {
        console.log(request);
        
        let result = response.template;
        result.success = true;
        result.message = "Some data";

        response.send(result);
    })

    node.createSubscription('std_msgs/msg/String', 'astra/core/feedback', (msg) => {
        for(let i in openSockets) {
            openSockets[i].emit('/core/feedback', msg.data);
        }
    })

    node.spin();
});

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON 0.0.0.0:${port}`);
});

// Ensure that the server stops when it is attempted to stop
process.on('SIGINT', function() {
    console.log( "\nShutting down from SIGINT (Ctrl-C)" );
    // Close the HTTP server
    server.close();
    // Shut the node down
    node.stop();
    // Exit the process, stop the server
    process.exit(0);
});