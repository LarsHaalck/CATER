# GUI Explanation:

## Keymaps

### Image viewing and zooming
Keymap                         | Function
--------                       | -------------
Scroll Wheel                   | Scroll image (when zoomed)
Control + Scroll Wheel         | Slow zoom into image
Control + Shift + Scroll Wheel | Fast zoom into image
Control + Shift + Scroll Wheel | Fast zoom into image
Left Arrow                     | Go to previous frame
Right Arrow                    | Go to next frame

### Manual Corrections

Keymap      | Function
--------    | -------------
left click  | Add or modify manual correction at current frame and position and move to next image
right click | Remove manual correction at current frame

### Labeling

Keymap            | Function
--------          | -------------
\<Hotkey\>        | Add label (and deactivating all other labels from the same group) as declared in label editor
Alt + \<Hotkey\>  | Toggle label "stickyness". If a label is sticky all frames that are visited from now on will implicitly get this label added.
CTRL + \<Hotkey\> | Set this label to all PREVIOUS frames until a frame with the same label occurs (asks for confirmation)
Backspace         | Clear all sticky labels


## "Unary-View":
### Quality
Color    | Explanation
-------- | -------------
green    | good quality frame
yellow   | medium quality frame
red      | bad quality frame
blue     | perfect quality, meaning manually corrected frame
gray     | undefined, meaning no information (usually before or after the start and end frame respectively)

A bad quality, that is not good or perfect does not mean that the tracking doesn't work.
These metrics have a lot of false positives and are only meant for an overview and are only critical if there are 50-100 frames in one chunk with a bad quality.

### Chunk-View
During optimization, frames are chunked and processed in parallel.
These chunks have alternating colors (darkgray and magenta).
If a chunk is optimized, the color is much lighter and there is a sound once a chunk (or all in the first run) is finished.

## Saving:
Optimize saves always
Manual unaries are only saved automatically, when optimized afterwards or when clicking File > Save.
Mike Super Button "saves" too, meaning clicking it again checks if files exists and loads them again (see special case "RAM full")

## Chunk Size:
Default chunk size is 100 but can be changed in preferences.

# FAQ:
## CATER crashed due to memory limits:
If memory is full in one of the early stages, try reducing the cache size from 200 to 100 in Tools > Preferences.

# Download
## Using AppImage:
Download the most recent AppImage from https://github.com/LarsHaalck/cater/releases listet under "Assets". You have to "allow executing file as program".  Right click on AppImage, then select Permissions tabe then select the Execute option. 

## Building from Source:
This project has a CMake superbuild structure, where the "root"-CMakeLists.txt lies in the folder `super`.

### Requirements:
```
g++ >= 10
cmake >= 3.8
```

### Optional but strongly encouraged:
Although they are both supplied via the superbuild, it is strongly encouraged to use the package manager of your Linux distro to install these packages.
This is especially true for OpenCV due to many possible optimizations, which can also be installed using `helper/build_opencv.sh` for Ubuntu 20.04 LTS.

```
OpenCV >= 4
Qt >= 6
```

**Ubuntu 22.04 supplies Qt (`qt6-base-dev` & `qt6-multimedia-dev`) and OpenCV (`libopencv-dev`) in a version that is recent enough.**

### Building
To build the full project from the superbuild simply run the following commands:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../super
make
```
the binary of the CATER-Gui should be compiled in `build/ui/gui/cater-gui`.
