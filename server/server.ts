import * as rclnodejs from 'rclnodejs';
import * as express from 'express';

const app = express();
const port = process.env.PORT || 8000;

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node('publisher_example_node');
    const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    publisher.publish(new Buffer("Hello ROS 2 from rclnodejs")); // wtf
    node.spin();
});

app.listen(port, () => {console.log(`Base Station Server started on port ${port}`)});