# Configuration files

In this document we list few examples of VSCode configuration files. We assume `WORKSPACE` to be the environment variable pointing to root of your workspace.

These files are located in the `.vscode` configuration folder. If you work on sub projects, you can have multiple configuration folders.

**c_cpp_properties.json**

```
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "/opt/ros/humble/include/**",
        "/usr/include/**"
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++14"
    }
  ],
  "version": 4
}
```

**launch.json**

This file is used to debug tests. In the example we use it to launch and debug the test called "test_config".

```
{
    "version": "0.2.0",
    "configurations": [
      {
        "name": "GDB: test_config",
        "type": "cppdbg",
        "request": "launch",
        "program": "${env:WORKSPACE}/build/my_project/test/test_binary",
        "args": [],
        "stopAtEntry": false,
        "cwd": "${env:WORKSPACE}",
        "environment": [],
        "externalConsole": false,
        "MIMode": "gdb",
        "preLaunchTask": "colcon: build",
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

**settings.json**

This file is used to configure Clangd for syntax higlighting, linting and file navigation.

```
{
    "cmake.configureOnOpen": false,
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "ros.distro": "humble",
    "workbench.colorCustomizations": {
        "tab.activeBorder": "#ffffff",
        "tab.activeBackground": "#373737"
    },
    "C_Cpp.intelliSenseEngine": "disabled",
    "clangd.arguments": [
        "-log=verbose",
        "-pretty",
        "--background-index",
        "--compile-commands-dir=${env:WORKSPACE}/build/"
    ]
}
```

**tasks.json**

This file is used to configure tasks that can belaunched from within VSCode like build, clean and test.

```
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "colcon: build",
            "type": "shell",
            "command": [
                "source /opt/ros/humble/setup.bash;",
                "colcon",
                "--log-base ${env:WORKSPACE}/log",
                "build",
                "--build-base ${env:WORKSPACE}/build",
                "--install-base ${env:WORKSPACE}/install",
                "--base-path ${env:WORKSPACE}/path-to-package",
                "--symlink-install",
                "--event-handlers console_cohesion+",
                "--cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=true"
            ],
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "colcon: clean",
            "type": "shell",
            "command": [
                "rm -rf ${env:WORKSPACE}/build;",
                "rm -rf ${env:WORKSPACE}/install;",
                "rm -rf ${env:WORKSPACE}/log;",
            ]
        }
    ]
}
```
