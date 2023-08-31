# VSCode and ROS2

This document describes how to configure VSCode to build and run ROS2 projects.

## Open VSCode

We assume you've installed VSCode without any additional extensions. In Ubuntu extensions are usually saved under the folder:

```
~/.vscode
```

To open an existing ROS2 repo, launch a terminal and move into your project workspace source folder. It must be the source folder, not the top workspace folder. For example

```
~/user/ros2_epick_gripper/src
```

Then, run the following command which will open that folder in VSCode.

```
code .
```

Once VSCode is up, you must to install Microsoft ROS extension and restart the editor.

<img width="80px" src="svg/ros_extension.svg">

This extension installs some dependencies like C/C++ and Python extensions.

<img width="80px" src="svg/cpp_extension.svg"><img width="80px" src="svg/python_extension.svg">

You may have noticed that VSCode create a folder .vscode with two files:

```
c_cpp_properties.json
settings.json
```

In the first file you may want to change the field `cppStandard` to `c++17`.

When you open a C++ file, VSCode may ask you to install the "C/C++ Extension pack". Please install it.

You can easily navigate between cpp and hpp files using the keyboard shortcut

`<alt> + o`

At the moment, you don't have a build task set up. In the `.vscode` folder you must create a file called `tasks.json` with the following content:

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

If you want to buils your repo, the only thing you have to do is to select "Run build task..." from the Terminal menu. The task will immediately find the build task in the above confuguration file. You can also use

`<ctrl> + <shift> + b`


## External links

There is a good introduction to VSCode and ROS2 here: https://www.youtube.com/watch?v=hf76VY0a5Fk