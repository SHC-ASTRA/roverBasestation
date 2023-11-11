import * as rclnodejs from 'rclnodejs';

export class RosNode extends rclnodejs.Node {
    PUBLISHER: rclnodejs.Publisher<'std_msgs/msg/String'>;

    constructor(outTopic: string) {
        // Initalize the base class constructor
        // to create a node of name "RosNode"
        super('RosNode');

        // Create ROS2 publisher
        // Possibly make use of JSON parsing
        this.PUBLISHER = this.createPublisher(
            'std_msgs/msg/String', // Type of topic message, stringify JSON
            outTopic // TOPIC NAME
        )
    }

    // Do not define the type of the message, oteherwise Typescript freaks out
    publishData(message): void {
        this.PUBLISHER.publish(message);
        console.log(`Publishing data: ${message}`);
    }
}