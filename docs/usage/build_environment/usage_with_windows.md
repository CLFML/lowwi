# Set-up build environment on Windows
Before you can compile and run the demo you need to install the following tools and libraries:

- Any C/C++ compiler (MSVC is recommended, but GCC works as well)
- CMake
- Ninja (**Optional**, but recommended as generating for MSBuild is very slow!)
- Any code editor (This guide will use VSCode, as it is free and easy to configure with CMake)

!!! note

    Although any recent enough C/C++ compiler can be used. 
    
    **This guide will only use the MSVC compiler!**

    This choice was made as this compiler is also used for all other CLFML projects.


## Installing MSVC16 (2019 edition) & CMake
The MSVC compiler (and CMake) can be installed by either installing VS BuildTools or Visual Studio 2019.

**This guide will use the VS BuildTools method, as we don't need the Visual Studio IDE**.

There are multiple ways you can download and install VS BuildTools 2019;

- Manually downloading and installing using this [link](https://aka.ms/vs/16/release/vs_buildtools.exe)
- Using chocolatey:
  ```choco install visualstudio2019buildtools```
- Using winget:
```winget install --id=Microsoft.VisualStudio.2019.BuildTools  -e```

### Manually installing VS Build Tools 2019
1. [Download and run this installer](https://aka.ms/vs/16/release/vs_buildtools.exe).
2. Select these options:
![Visual Studio Build Tools Options](img/vs_build_tools_options.png)
3. After installation, reboot your machine!


## Installing Ninja
1. [Download the latest Ninja release for Windows!](https://github.com/ninja-build/ninja/releases)
2. Unzip this ninja-win.zip to `C:\ninja-win`
3. Open the environment variables editor using the Windows Startup Menu ([Try this guide if you can't find it](https://www.imatest.com/docs/editing-system-environment-variables/#Windows))
4. Add the `C:\ninja-win` path to the PATH variable;
5. Open a commandline window and check if Ninja is correctly installed by running the `ninja` command!


## Installing VSCode (with plugins)
VSCode is an easy to use code-editor with CMake support (using the CMake Tools plugin). 

To set-up VSCode the follow these steps:

1. [Download and install VSCode using the installer](https://code.visualstudio.com/download)
2. Follow the initial set-up wizard in vscode (if freshly installed)
3. Download and install this plugin pack:
    - C/C++ Extension Pack (Microsoft)

## Compiling and running the example
The library contains an example demonstrating the usage and functionality of this library. 

To compile and run this example:

1. Clone this repo:
```
git clone https://github.com/CLFML/lowwi.git
```

2. Open the cloned repo folder in vscode; `File->Open Folder`

3. Select Ninja as build generator by pressing **CRTL+SHIFT+P**->**"CMake: Open CMake Tools Extension Settings"**->**"@ext:ms-vscode.cmake-tools generator"**
   Now type Ninja (with capital N into the generator field!).
   ![CMake extension tool settings; Generator](img/vscode_cmake_generator.png)

4. Select the `MSVC amd64 kit`by pressing CTRL+SHIFT+p and selecting `CMake: Select a kit`.

5. CMake will now configure; By default it will configure as Debug build, this has a significant performance hit.
   To change to release with debug info (which has optimizations turned on, but is still debuggable). Press CTRL+SHIFT+p again and enter `CMake: Select Variant`-> `RelWithDebInfo`
   ![Variant](img/build_variant.png)

6. Let CMake Finish configuring your build configuration. **Then click on the Play button on the blue bar on the bottom of screen**, CMake might ask which target to launch, select the `Lowwi_demo_mic` target.
   ![Launch target](img/launch_target.png)

7. After build is finished, it will launch the demo which uses your microphone to detect the **"Hey Mycroft"** wakeword.