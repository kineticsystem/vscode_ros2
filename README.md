# ROS2 and VSCode

<img height="70px" src="svg/visual_studio_code.svg">

This documentation outlines the procedures for setting up Visual Studio Code (VSCode) to build and execute ROS2 projects effectively.

## Table of Contents

- [ROS2 and VSCode](#ros2-and-vscode)
  - [Table of Contents](#table-of-contents)
  - [Initialization of Visual Studio Code](#initialization-of-visual-studio-code)
  - [Working in C++](#working-in-c)
    - [Navigation and Shortcuts](#navigation-and-shortcuts)
    - [Colcon build](#colcon-build)
    - [Debugging tests](#debugging-tests)
  - [Working in Python](#working-in-python)
    - [Debugging Python files](#debugging-python-files)
  - [Remote development over SSH](#remote-development-over-ssh)
  - [Development on Docker](#development-on-docker)
  - [Additional extensions](#additional-extensions)
    - [General](#general)
    - [Python](#python)
  - [TODO](#todo)
  - [Additional Resources](#additional-resources)

## Initialization of Visual Studio Code

For the purpose of this guide, it is presumed that Visual Studio Code has already been installed on your system without any additional extensions. In an Ubuntu environment, the extensions are commonly stored in the following directory:

```
~/.vscode
```

Upon launching VSCode, proceed to install the Microsoft ROS extension, followed by a restart of the editor.

<img height="80px" src="svg/ros_extension.svg">

This extension will facilitate the installation of requisite dependencies such as Microsoft C/C++ and Microsoft Python extensions.

<img height="80px" src="svg/cpp_extension.svg"><img height="80px" src="svg/python_extension.svg">

Install the "C/C++ Extension Pack" which provides Intellisense and C++ file navigation. This will install the CMake extension from twsx too.

<img height="80px" src="svg/cmake_extension.svg">

You may need to enable Intellisense in your VSCode Preferences Settings.

Restart VSCode and open the root of your ROS2 project repository where `build`, `install`, `log` and `src` folders are usually located. For instance:

```
~/user/my_project
```

## Working in C++

If you open an existing ROS2 project that contains C++ source code, you may observe that VSCode generates a `.vscode` folder containing two configuration files:

```
c_cpp_properties.json
settings.json
```

If `c_cpp_properties.json` is not created, you can create a new one by opening the command palette and typing "C++: Edit Configurations (UI)".
Update it to match roughly the following content:

```json
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "${workspaceFolder}/**",
        "/opt/ros/humble/include/**",
        "/usr/include/**"
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++17"
    }
  ],
  "version": 4
}
```

### Navigation and Shortcuts

You may conveniently toggle between `.cpp` and `.hpp` files using the following keyboard shortcut:

`Alt + O`

### Colcon build

If you want to run Colcon from VSCode, always remember to open the root of your repository where `build`, `install`, `log` and `src` folders are usually located.

At this stage, no task has been defined. To rectify this, create a `tasks.json` file within the `.vscode` folder and populate it with the following content:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "source /opt/ros/humble/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install --event-handlers log-"
        },
        {
            "label": "clean",
            "type": "shell",
            "command": "rm -rf build/ install/ log/",
            "problemMatcher": []
        },
        {
            "label": "test",
            "type": "shell",
            "command": "source /opt/ros/humble/setup.bash; source install/setup.bash; colcon test && colcon test-result --verbose"
        }
    ]
}
```

Remember to modify the following line according to your ROS2 distribution. In this way, you will be able to execute Colcon commands from within VSCode:

`source /opt/ros/humble/setup.bash`

To execute a build, navigate to the "Run build task..." option within the Terminal menu. The system will automatically locate and initiate the build task specified in the aforementioned `tasks.json` file. Alternatively, use the keyboard shortcut:

`Ctrl + Shift + B`

### Debugging tests

Another useful extension is "C++ TestMate" to launch and debug GTests directly within VSCode.

<img height="80px" src="svg/testmate_extension.svg">

Please note that for this extension to work correctly, you may need to source your ROS repository before starting up VSCode from the terminal.

`source install/setup.bash; code .`

To debug your test, you must create a `launch.json` file inside your `.vscode` directory. The file is automatically created for you when you debug your first test.

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "enter program name, for example ${workspaceFolder}/a.out",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${fileDirname}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

## Working in Python

When you open an existing ROS2 Python project, IntelliSense may not find ROS2 Python modules, or your local package modules. To solve the issue, create a file `settings.json` with a content matching roughly the following:

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages", 
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/my_project/build/package1",
        "/my_project/build/package2"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages", 
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/my_project/build/package1",
        "/my_project/build/package2"
    ]
}
```

### Debugging Python files

To debug a selected Python file, you must create a `launch.json` file inside your `.vscode` directory. The file is automatically created for you when you debug your first test with the following content.

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "args": [""], // Only if you need parameters
            "console": "integratedTerminal",
            "justMyCode": true
        }
    ]
}
```

To debug a ROS2 launch file, you must add this code at its end:

```python
def main():
    ls = launch.LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()
```

With this code, we transform the launch file into a standard executable file that we can debug as a normal python file.

Using an "opaque function" allows you to debug the values of launch parameters. However, together with the previous change, you'll need to modify the launch file as demonstrated in the following example.

```python
def launch_setup(context, *args, **kwargs):
    my_param = LaunchConfiguration("param_name")

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_name",
                default_value="param_value",
                description=("param_description"),
            ),
            ...,
            OpaqueFunction(function=launch_setup),
        ]
    )
```

In VSCode, we can add a breakpoint and display the content of this variable with a watch:

```python
my_param.perform(context)
```

## Remote development over SSH

VSCode can be used to develop remotely over SSH. You must install an extension called Microsoft Remote SSH. This extension will also add a button on the left toolbar to display all available Docker containers.

<img height="80px" src="svg/ssh_extension.svg">

Open the command palette and type: "Remote-SSH: Connect to Host...".

## Development on Docker

VSCode can be used to develop on Docker. You must install an extension called Microsoft Dev Containers which you can use to connect to running containers. You can easily install your local VSCode extensions into the container.

<img height="80px" src="svg/dev_container_extension.svg">

More information here: https://code.visualstudio.com/docs/devcontainers/attach-container

## Additional extensions

### General

* **Back & Forth:** Extension to add go back/forward buttons to the title bar for easier navigation through recent edit locations and opened files.

* **Debug Visualizer:** Extension for visualizing data structures while debugging. Like the watch view, but with rich visualizations of the watched value.

* **Error Lens:** Extension to better display errors in the Python code.

* **Markdown All in One:** Extension for Markdown advanced editing. It adds document outline, automatic table of contents, etc.

* **Protobuf (Protocol Buffers):** Extension to add Protobuf support, powered by Pbkit language server.

* **UMLet:** Draw UML diagram inside VSCode.

### Python

* **Astral Software Ruff:** Support for the Ruff linter.

* **Debug Launcher:** Start debugging, without having to define any tasks or launch configurations, even from the terminal.

* **Microsoft Black Formatter:** An automatic Python code formatted.

* **Microsoft Pylint**:  Support for the Pylint linter.

* **Microsoft Pylance**: This is a powerful tool that enhances the Python development experience in Visual Studio Code by providing tools for code analysis, error checking, code navigation, and code completion. 

## TODO

- Execute and debug Python launch files.

## Additional Resources

For additional information about ROS2 and VSCode, you may refer to the following tutorials:

- https://www.youtube.com/watch?v=hf76VY0a5Fk
- https://www.allisonthackston.com/articles/vscode-docker-ros2.html
