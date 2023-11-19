/* Base Imports */
import * as path from 'path'
/* RCLNodejs */
import * as rclnodejs from 'rclnodejs';
/* Custom ROS2 Node Implementation */
import {RosNode} from './ros'
/* HTTP */
import * as http from 'http';
import {Server, IncomingMessage, ServerResponse} from 'http';
/* Express */
import * as express from 'express';
const app = express();

// const app = express();
const port: number = Number(process.env.PORT) || 8000;

rclnodejs.init().then(() => {
    // Create the node
    const node = new RosNode();
    node.createTopic('astra/core/control');
    setInterval(
        () => node.publishData('astra/core/control', 'Hello from the Basestation!'),
        5000        
    );
    node.spin();
});

// Host the build react page root
app.use(
    '/',
    express.static(path.join(__dirname, "..", "react-app", "build"))
);

app.get(
    '/api/:id', (req, res) => {
        return res.send(`API in Progress. Received ID ${req.params.id}`);
    }
);

var server:Server = http.createServer(app);

server.listen(port, '0.0.0.0', () => {
	console.log(`LISTENING ON ${port}`);
});
