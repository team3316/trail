Trail
===

> D-Bug's trajectory planning utility for the 2019 Destination: Deep Space season

## Cloning
The project uses `googletest` and `Eigen` as Git submodules, which means that in order to clone the project you need to use `--recurse-submodules`:

```bash
git clone --recurse-submodules https://github.com/team3316/trail.git
```

## Compiling
The project uses CMake in order to build. First, create a `build` directory and `cmake` inside of it:
```bash
mkdir build && cd build && cmake ..
```

Then just `make` as usual:
```bash
make
```

## Testing
In order to test the project, follow the steps above for compiling but use `make test` instead.

## Usage
In order to generate profiles, the program requires two JSON files: a robot configuration and a profile definition.

### Creating a robot configuration JSON
This file holds relevant information about the robot the profile is generated for. Currently, the profile generation supports only robots that use tank drive (no swerve or mechanum or anything crazy like that), and ones that have the same transmission in each side and that have the same motors all around (no 2 CIM and 1 Mini CIM configurations allowed). 

The file should include the following properties:
 - `mass` - The mass of the robot in kg.
 - `base-width` - The wheelbase width (wheel center-to-center) in m.
 - `free-speed` - The free speed of the transmission in m/s.
 - `stall-torque` - The stall torque of the motors in the transmission in Nm.
 - `gear-ratio` - The overall gear ratio of the transmission as output over input.
 - `wheel-radius` - The radius of the wheels on the drivetrain in m.
 - `motors` - The **total** number of motors on the drivetrain.

Other drivetrain types and transmission configurations are planned for a future release.

An example JSON is provided in the `example` folder.

### Creating a profile definition JSON
This file holds relevant information about the motion of the robot on the field.

The file should include the following properties:
 - `name` - The name of this motion sequence. This will be used for the name of the CSV export file and is required.
 - `waypoints` - An array of locations of the robot on the field. Each object inside the `waypoints` array has to include these properties:
    - `point` - A 2D array, with the x-y location of the _center_ of the robot on the field.
    - `heading` - A number representing the robot's heading angle in angles (not radians).

Other inputs may be added in a future release.

An example JSON is provided in the `example` folder.

### Using the tool
After downloading and compiling, you should have an executable called `trail-cli` in the `build` folder.
Using the JSON files you just created (let's call the robot one `robot.json` and the profile `path.json`), execute the following command:
```bash
./trail-cli robot.json path.json
```
This should create a new CSV file in the current directory with the name of the path and the profile data inside of it.

And that's basically it!

Another option is to output the whole profile for viewing in [Desmos](https://desmos.com/calculator). In order to do that,
you just need to use the `-d` or `--desmos` flag:
```bash
./trail-cli robot.json path.json --desmos
```
Then just copy and paste the output into Desmos and you should be able to see the graphs of the profile (distance vs time, x vs y...)
