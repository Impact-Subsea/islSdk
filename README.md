# Impact Subsea Sdk

The Software Development Kit (SDK) is an open source software library that simplifies the development of an application intended to communicate with Impact Subsea products. The SDK provides the following:<br>
- Ability to open, close and configure the computer’s serial and network ports.
- Interact with Serial over LAN (SOL) or (network to serial adapters).
- Automatically discover Impact Subsea devices connected to the computer.
- Automatically detect NMEA devices, such as a GPS connected to the computer serial ports.
- Dynamically creates objects to manage each discovered device and allow the user’s application to interact with the device via this object.
- Log all connected Impact Subsea devices and GPS data to a log file.
- Play back of log files.

The SDK is written in C++17 and cmake is used for compiling the project. It has been written in a cross-platform way with embedded credit card computers in mind, such as the Raspberry Pi and Beagle Bone type systems. The minimum hardware specification is:<br>
- 100Mhz 32-bit processor with FPU (Floating Point Unit)
- 32MB RAM

## Compiling

### Using Microsoft Visual Studios with cmake
1. Open Microsoft Visual Studios and click `Continue without code`.
2. Open the `CmakeLists.txt` file from the menu `File->Open->CMake..`
3. Select the startup object from the dropdown green compile and run button.
4. Click the Compile / Run button.

### Linux, Windows using cmake

1. Make sure you have `cmake` installed.

2. Clone the git repo onto your local storage.

3. Change into root repo directory:

    ```
    $ cd islSdk
    ```

4. Create a new build directory and change into it:

    ```bash
    $ mkdir build
    $ cd build
    ```

5. Run cmake on the parent directory to generate makefile:

    ```bash
    $ cmake -DCMAKE_BUILD_TYPE=Debug ..
    or
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    ```

6. Run make on the generated makefile to generate the static library:

    ```bash
    $ make
    ```

- If on Linux make sure the app has permission to access the serial ports:

    ```bash
    $ sudo usermod -a -G dialout YOUR_USER_NAME
    ```

## Documentation

Open the doc/documentation.html file for more sdk documentation.
