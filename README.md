DroneCAN DSDL definitions
=========================

This repository contains the DSDL definitions of the standard DroneCAN
messages and services.

This DSDL is based on DroneCAN v1

For information about the Message type IDs (including how the unqiue IDs are allocated), see [**id distribution**](https://dronecan.github.io/Specification/5._Application_level_conventions/#id-distribution)

Message ID
----------

For broadcast messages the following should be used:

 - [0, 20000) Standard message types
 - [20000, 21000) Vendor-specific message types
 - [21000, 65536) Reserved for future use

For service types, the following should be used:

 - [0, 100) Standard service types
 - [100, 200) Reserved for future use
 - [200, 256) Vendor-specific service types

Vendor message ID allocations
------------------------------

| Range           | Namespace           | Owner     | Type    |
|-----------------|---------------------|-----------|---------|
| 20000 - 20019   | ardupilot.*         | ArduPilot | Project |
| 20020 - 20039   | com.volz.*          | Volz      | Company |
| 20050 - 20069   | com.hobbywing.*     | HobbyWing | Company |
| 20200 - 20219   | com.hex.*           | Hex       | Company |
| 20300 - 20319   | com.xacti.*         | Xacti     | Company |
| 20720 - 20799   | ardupilot.*         | ArduPilot | Project |
| 20800 - 20899   | px4.*               | PX4       | Project |
| 20900 - 20919   | com.auterion.*      | Auterion  | Company |

Legacy allocations (IDs outside the blocks above, predating this table):

| IDs in use          | Namespace           |
|---------------------|---------------------|
| 20009               | mppt.*              |
| 20013               | com.hobbywing.*     |
| 20030 - 20035       | dronecan.remoteid.* |
| 20100               | com.hobbywing.*     |
| 20300               | cuav.*              |
| 20402, 20407, 20554 | com.xacti.*         |
| 21910               | ardupilot.*         |

* [**DroneCAN website**](http://dronecan.org)
