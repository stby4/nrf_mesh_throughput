# Light switch example

> This example has only been tested with the nRF52840 series.

Throughput test based on Nordics light switch example.


## Setup
- Download and install the [generic message model for mesh](https://github.com/stby4/nrf_mesh_generic_message_model).
- Go to the 'nrf5SDKforMeshv310src\examples' folder
- `git clone git@github.com:stby4/nrf_mesh_throughput.git throughput`
- Open the 'throughput' folder. You will find seperate projects for the client and the server.



## Hardware requirements

You need at least two supported boards for this example:

- One nRF52840 development board for the client.
- One or more nRF52840 development boards for the servers.

Additionally, you need the following:
- _nRF Mesh_ app for [Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.nrfmeshprovisioner) or [iOS](https://apps.apple.com/us/app/nrf-mesh/id1380726771) if you decide to provision using the application.


## Testing

To test the throughput, build the examples with Segger Embedded Studio.

After building is complete, use the nRF Mesh mobile app to provision the mesh devices.
Once the provisioning is complete, you can start the throughput test.



### Evaluating using the nRF Mesh mobile app

1. Flash the examples by following the instructions:
    1. Erase the flash of your development boards and program the SoftDevice.
    2. Flash the client firmware on individual boards and the server firmware on other board or boards.
2. Open the nRF Mesh mobile app.
3. Provision the nodes. The client board is `nRF5x Message Client`,
the server board is `nRF5x Message Server`.
4. Bind the Generic OnOff client and server model instances on the nodes with the same app key:
    1. Select the Network tab.
    2. On the server board tile, tap the **Configure** button to open Node Configuration.
    3. Expand the Elements section and tap **Generic OnOff Server**.
    4. In the Bound App Keys section, tap the **Bind Key** button and select the app key.
    5. On the client board tile, tap the **Configure** button to open Node Configuration.
    6. Expand the Elements section and tap **Generic OnOff Client**.
    7. In the Bound App Keys section, tap the **Bind Key** button and select the app key.
6. In the client Node Configuration, expand the Elements section. 
7. Select the first Generic OnOff Client model instance.
8. In the Publish section, set the Publish Address to the unicast address of any server node. This configures the client example as follows:
        - The Button 1 on the client board will start the throughput test with the selected server.
        
> You can also set the Publish Address of the Generic OnOff model instance on the server board.
> If you set the address to any group address, the server example will be configured as follows:
>    - The Button 1 will send a test message to the corresponding group of client boards.
> 
> Remember to subscribe the client boards to events on the group address.
