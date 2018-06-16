# Houdini-NvFlex
simple integration of Nvidia Flex technology into SideFX Houdini

[see animated preview here](https://i.imgur.com/rKzsk49.gifv)

![preview](https://i.imgur.com/rKzsk49.gif)

## installation (Windows):
* copy dll from x64 folder (nvFlexDop_165.dll for houdini 16.5, nvFlexDop_160.dll for houdini 16.0) to any dso location, like for example into **C:\Users\\\<YOUR USER\>\Documents\houdini16.5\dso\\**
* copy all *.otl files from otls folder to your otls folder, like for example into **C:\Users\\\<YOUR USER\>\Documents\houdini16.5\otls\\**
* copy all *.dll files from dll folder to any path in your system PATH environment variable.
The most straightforward way would be to copy them into your **system32** folder, like **C:\Windows\system32\\** ,
but i would suggest to copy them into a new folder, for example **C:\mystuff\bin\nvflex\\** and add that path to system PATH variable. 
Please, see [here](https://superuser.com/questions/949560/how-do-i-set-system-environment-variables-in-windows-10) for instructions how to access environment variables.
just add your folder to the end of the **PATH** variable. So with example folder above it will looks like **\<CONTENT OF YOUR PATH VARIABLE\>;C:\mystuff\bin\nvflex\\**

That should do it.


If you encounter problems, like houdini crashes on startup or plugin not working in general: try [this work in progress branch](https://github.com/pedohorse/Houdini-NvFlex/tree/work-in-progress)

it will at least print out the problem into the console

If the plugin does not load and no console message is printed, but houdini itself launches - you probably forgot to make some of dlls from the **bin** folder available (see pt.3 of installation guide, and make sure you have microsoft vc 2015 redistributable installed (it would be fun if you don't cuz houdini needs it to launch as well))

## Examples

please, see hip files in **examples** folder
