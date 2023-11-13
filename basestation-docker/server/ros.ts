import * as rclnodejs from 'rclnodejs';

export class RosNode extends rclnodejs.Node {
    PUBLISHER: rclnodejs.Publisher<'std_msgs/msg/String'>;
    SERVICE;

    constructor(outTopic: string, serviceCallback: (request, response) => void) {
        // Initalize the base class constructor
        // to create a node of name "RosNode"
        super('RosNode');

        // Create ROS2 publisher
        // Possibly make use of JSON parsing
        this.PUBLISHER = this.createPublisher(
            'std_msgs/msg/String', // Type of topic message, stringify JSON
            outTopic // TOPIC NAME
        );

        // Create ROS2 service server
        this.SERVICE = this.createService('health_interface/srv/HealthReport', 'astra/core/health', serviceCallback)
    }

    // Do not define the type of the message, oteherwise Typescript freaks out
    publishData(message): void {
        this.PUBLISHER.publish(message);
        console.log(`Publishing data: ${message}`);
    }
}