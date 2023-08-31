# VSCode and ROS2

This documentation outlines the procedures for setting up Visual Studio Code (VSCode) to effectively build and execute ROS2 projects.

## Initialization of Visual Studio Code

For the purpose of this guide, it is presumed that Visual Studio Code has already been installed on your system without any additional extensions. In an Ubuntu environment, the extensions are commonly stored in the following directory:

```
~/.vscode
```

To open an existing ROS2 repository, initiate a terminal and navigate to the src directory within your ROS2 workspace. It is crucial to ensure you are in the src directory, rather than the root workspace directory. For instance:

```
~/user/ros2_epick_gripper/src
```

Execute the following command to open this directory in VSCode:

```
code .
```

Upon launching VSCode, proceed to install the Microsoft ROS extension, followed by a restart of the editor.

<img width="80px" src="svg/ros_extension.svg">

This extension will facilitate the installation of requisite dependencies such as the C/C++ and Python extensions.

<img width="80px" src="svg/cpp_extension.svg"><img width="80px" src="svg/python_extension.svg">

## Configuration Files

You may observe that VSCode generates a `.vscode` folder containing two configuration files:

```
c_cpp_properties.json
settings.json
```

Within `c_cpp_properties.json`, consider updating the `cppStandard` field to `c++17`.

When you open a C++ file, VSCode may prompt you to install the "C/C++ Extension Pack". Please comply with this request.

## Navigation and Shortcuts

You may conveniently toggle between `.cpp` and `.hpp` files using the following keyboard shortcut:

`Alt + O`

## Build Task Configuration

At this stage, no build task has been defined. To rectify this, create a `tasks.json` file within the `.vscode` folder and populate it with the following content:

```
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "build",
      "type": "shell",
      "command": "colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
    },
    {
      "label": "test",
      "type": "shell",
      "command": "colcon test && colcon test-result"
    }
  ]
}
```

To execute a build, navigate to the "Run build task..." option within the Terminal menu. The system will automatically locate and initiate the build task as specified in the aforementioned `tasks.json` file. Alternatively, use the keyboard shortcut:

`Ctrl + Shift + B

## TODO

* Execute and debug GTests.

* Execute and debug Python launch files.

* Remote developing.

* Developing with Docker.

## Additional Resources

For an in-depth introduction to ROS2 and VSCode, you may refer to the following video tutorial: https://www.youtube.com/watch?v=hf76VY0a5Fk