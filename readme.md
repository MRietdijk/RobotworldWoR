# Robotworld install

This manual is for installing and configuring the Robotworld application.

The following steps need to be taken:

- `cd robotworld/linux`
- `../configure --with-cxx=17`
- `make`
- `./src/robotworld`

Now robotworld is running.

## Making changes to the configuration

For robotworld there are a couple of configuration variables:

 - stdev-compass
 - stdev-odometer
 - stdev-lidar

These configurations are found in `${project-folder}/config/configuration.xml`.

Here you can change the different settings.