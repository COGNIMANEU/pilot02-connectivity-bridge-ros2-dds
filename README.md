# Pilot02 Connectivity ROS2 DDS Bridge 

This repository provides a bidirectional communication bridge between ROS2 nodes in distributed environments, enabling seamless interoperability between nodes on different ROS2 domains via DDS (Data Distribution Service). Built upon the zenoh-bridge-ros2dds component from [zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git), it allows for flexible configuration and exchange of messages between ROS2 systems across multiple domains.

The bridge facilitates a seamless exchange of messages by allowing the configuration of topics over which the communication bridge operates. This enables ROS2 nodes on different domains to interact effectively without requiring direct network access between them.

Features:
- **Bidirectional communication** between ROS2 nodes across different ROS2 domains.
- **Topic configuration** for establishing communication bridges between specified topics.
- **Docker Compose-based test automation** that verifies interoperability between a publisher and a subscriber in different ROS2 domains via a bridge client and a bridge peer.
- Based on the **zenoh-bridge-ros2dds** project, leveraging Zenoh for robust data distribution.

The repository includes the following components:
- **Configuration files** for setting up topics and managing ROS2 domain IDs.
- **Docker Compose environment** that automates testing and validation of the bridge's functionality by simulating a public publisher and subscriber in different domains, connected through a bridge client and peer.
- **Zenoh** integration for efficient message transport between ROS2 systems.

## Guidelines for build and test the component 

### 1. **Build the Main Docker Image:**

In this step, we build the Docker image using the provided `Dockerfile`. The image is named `pilot02-connectivity-bridge-ros2-dds`.

```bash
cd src
docker build -t pilot02-connectivity-bridge-ros2-dds .
```
Make sure the path to your configuration and launch files is correctly mapped to the Docker container.

### 2. **Run the ROS 2 Container:**

After building the Docker image, you can run the container using the following command:

```bash
docker run -e CONFIG_FILE=your_custom_config_file pilot02-connectivity-bridge-ros2-dds
```

This will start the container and launch the MQTT bridge with the configuration given.

### 3. **Build and Run the test automation:**

Test automation is integrated by docker-compose file:

Run: 
```bash
docker-compose up --build
```

After execution, you will be able to see how the messages from the publisher are received by the subscriber, even though they are in different domains, through the bridges.

```python
publisher-1    | [INFO] [1744275693.029431130] [hello_publisher]: Publishing: 'Hello world 0'
subscriber-1   | [INFO] [1744275693.030953128] [hello_subscriber]: Received: 'Hello world 0'
subscriber-1   | [INFO] [1744275694.010909020] [hello_subscriber]: Received: 'Hello world 1'
publisher-1    | [INFO] [1744275694.013533761] [hello_publisher]: Publishing: 'Hello world 1'
publisher-1    | [INFO] [1744275695.011237237] [hello_publisher]: Publishing: 'Hello world 2'
subscriber-1   | [INFO] [1744275695.014337847] [hello_subscriber]: Received: 'Hello world 2'
publisher-1    | [INFO] [1744275696.012536117] [hello_publisher]: Publishing: 'Hello world 3'
subscriber-1   | [INFO] [1744275696.013083447] [hello_subscriber]: Received: 'Hello world 3'
publisher-1    | [INFO] [1744275697.020725502] [hello_publisher]: Publishing: 'Hello world 4'
subscriber-1   | [INFO] [1744275697.020860969] [hello_subscriber]: Received: 'Hello world 4'
[rest of logs...]
```

## Bridge configuration

Configuration for the ROS2 DDS bridge is provided via json files.

### scope
Type: `String`  
Description: A prefix added to all routed DDS topics when mapped to a Zenoh resource. This is useful to avoid conflicts when several DDS systems with the same topic names are routed via Zenoh.  
Default: `"demo"`

### domain
Type: `Integer`  
Description: The DDS Domain ID. By default, it is set to `0`, or `$ROS_DOMAIN_ID` if the environment variable is defined.  
Default: `0`

### localhost_only
Type: `Boolean`  
Description: If set to `true`, DDS discovery and traffic will occur only on the localhost interface (`127.0.0.1`).  
Default: `false` (unless the `ROS_LOCALHOST_ONLY=1` environment variable is defined)

### shm_enabled
Type: `Boolean`  
Description: If set to `true`, the DDS implementation will use the Iceoryx PSMX plugin. The bridge must be built with the `'dds_shm'` feature enabled.  
Default: `false`

### allow
Type: `Array of strings`  
Description: A list of regular expressions matching the partitions/topics that must be routed via Zenoh. If both `allow` and `deny` are set, a partition/topic will be allowed if it matches only the `allow` expression.  
Default: All partitions and topics are allowed unless specified.  
Example:
```json
allow: ["cmd_vel", "rosout"]
```

### max_frequencies
Type: Array of strings
Description: Specifies the maximum frequency for data routing over Zenoh for a set of topics. The strings must follow the format <regex>=<float>, where <regex> matches the set of partitions/topics and <float> is the maximum frequency in Hertz.
Default: None
Example: 
```json
max_frequencies: ["diagnostic.*=10", "rosout=5"]
```

### generalise_subs
Type: Array of strings
Description: A list of key expressions to use for generalizing subscriptions.

### generalise_pubs
Type: Array of strings
Description: A list of key expressions to use for generalizing publications.

### forward_discovery
Type: Boolean
Description: When true, discovery information is forwarded to remote plugins/bridges rather than creating a local route.
Default: false

### reliable_routes_blocking
Type: Boolean
Description: When true, publications from a RELIABLE DDS Writer will be routed using CongestionControl::Block, meaning routing will be blocked during network congestion.
Default: true

### queries_timeout
Type: Float
Description: The timeout duration in seconds for queries made by the bridge to remote bridges for discovery information and historical data for TRANSIENT_LOCAL DDS Readers it serves.
Default: 5.0

### work_thread_num
Type: Integer
Description: The number of worker threads in the asynchronous runtime.
Default: 2

### max_block_thread_num
Type: Integer
Description: The number of blocking threads in the asynchronous runtime.
Default: 50

### id
Type: String
Description: The unique identifier for the Zenoh bridge. If not set, a random UUIDv4 will be used. This ID must be unique in the Zenoh network.
Example: "A00001"

### mode
Type: String
Description: The mode of the bridge. It can either be peer or client.
Default: "client"

### connect
Type: Object
Description: A list of endpoints the bridge will connect to. This tells Zenoh which router/peer the bridge should connect to at startup.
Example: 
``` json
connect: {
  endpoints: ["tcp/bridge-server:7447"]
}
```

### listen
Type: Object
Description: A list of endpoints that the bridge will listen to. This tells Zenoh which endpoints other routers, peers, or clients can use to establish a Zenoh session.
Example:
``` json
listen: {
  endpoints: ["tcp/0.0.0.0:7447"]
}
``` 

## Contributing

Refer to [zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git) repository for zenoh changes. Feel free to open issues or submit pull requests. Contributions are welcome!

## License

This project is licensed under the Apache2 and Eclipse Public License v2.0 - see the [LICENSE](LICENSE) file for details.