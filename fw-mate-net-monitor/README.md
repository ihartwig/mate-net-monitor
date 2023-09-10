# fw-mate-net-monitor

A Particle project named fw-mate-net-monitor

This project implements the uMATE library on top of Particle Boron with a mate-net-monitor adapter board to act like the status screen on a MATE controller and save this data to a cloud database.

More details on the [MATE-to-MX protocol](https://github.com/jorticus/pymate/blob/master/doc/protocol/Protocol.md) have been documented by jorticus - thank you! The simple implementation here requests the device type and status page from port 0 every 5 minutes. The bus is scanned more frequently (approx. every 1 second) to drive MATE Net LEDs.

#### LED Indicators:

MATE Net Port Green: powered up (+24V) from this port

MATE Net Port Orange: solid for running port scan; blinking when at least one MATE device is found during last scan

Mag Net Green: ports A and B are connected correctly based on the +14V power; power is not drawn from this port

Mag Net Orange: not implemented

Mag Net Red: ports A and B are swapped

#### Limitations:
* always asks for the MX Charger status on port 0 if it is not discovered on another port
* * mate bus scan to discover devices behind Hub is implemented, but not tested with a Hub
* Mag net ports are not implemented - should have no impact since that is a multi-drop bus

#### Topology:

```
Particle Cloud
       |
      LTE
       |               MAX3100 UART &
     Boron --- SPI --- Opto-Isolation --- MATE-net --- MX Charger
                       3.3V       24V
```

#### Library modifications:

* uMate has extra #define OUT statements commented for conflict with Particle libs
* MAX3100Serial9b includes a compatible Stream9b.h to the one used in uMate


## Welcome to your project!

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for fw-mate-net-monitor.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h`, `.cpp` & `library.properties` files for your library there. Read the [Firmware Libraries guide](https://docs.particle.io/guide/tools-and-features/libraries/) for more details on how to develop libraries. Note that all contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
