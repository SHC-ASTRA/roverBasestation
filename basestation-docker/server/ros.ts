import * as rclnodejs from 'rclnodejs';

export class RosNode extends rclnodejs.Node  {
    publishers = {};
    corehealthService;
    armhealthService;

    constructor() {
        // outTopic: string, serviceCallback: (request, response) => void
        // Initalize the base class constructor
        // to create a node of name "astra_base"
        super('astra_base');
    }

    createTopic(topicName /* string */) /* rclnodejs.Publisher<'std_msgs/msg/String'> */ {
        this.publishers[topicName] = this.createPublisher(
            'std_msgs/msg/String',
            topicName
        );
        return this.publishers[topicName];
    }

    // Do not define the type of the message, otherwise Typescript freaks out
    publishData(topicName, message) {
        this.publishers[topicName].publish(message);
        console.log(`Publishing data: ${message}`);
    }

    // Health packet service host for confirming
    // a connection to the basestation
    // The callback should just return a simple timestamp
    // as defined by the ROS2 interface

    // In order to test the functionality of this service, make use of
    // the below command
    // `ros2 service call /astra/MODULE/health std_srvs/srv/Trigger`


    initalizeHealthPackets(serviceCallback: (request, response) => void) {
        this.corehealthService = this.createService(
            {package: 'std_srvs', type: 'srv', name: 'Trigger'},
            '/astra/core/health',
            serviceCallback
        );
        this.armhealthService = this.createService(
            {package: 'std_srvs', type: 'srv', name: 'Trigger'},
            '/astra/arm/health',
            serviceCallback
        )
    }

    
}