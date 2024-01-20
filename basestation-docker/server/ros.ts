import * as rclnodejs from 'rclnodejs';

export class RosNode extends rclnodejs.Node {
    publishers = {};
    healthService;

    constructor() {
        // outTopic: string, serviceCallback: (request, response) => void
        // Initalize the base class constructor
        // to create a node of name "astra_base"
        super('astra_base');
    }

    createTopic(topicName: string): rclnodejs.Publisher<'std_msgs/msg/String'> {
        this.publishers[topicName] = this.createPublisher(
            'std_msgs/msg/String',
            topicName
        );
        return this.publishers[topicName];
    }

    // Do not define the type of the message, otherwise Typescript freaks out
    publishData(topicName, message): void {
        this.publishers[topicName].publish(message);
        console.log(`Publishing data: ${message}`);
    }

    // Health packet service host for confirming
    // a connection to the basestation
    // The callback should just return a simple timestamp
    // as defined by the ROS2 interface
    initalizeHealthPackets(serviceCallback: (request, response) => void) {
        /* this.healthService = this.createService(
            'health_interface/srv/HealthReport',
            'astra/core/health',
            serviceCallback
        ); */
    }
}