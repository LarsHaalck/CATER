# Using AppImage:
Download the most recent AppImage from https://github.com/LarsHaalck/habitrack/releases listet under "Assets".

# Building from Source:
This project has a CMake superbuild structure, where the "root"-CMakeLists.txt lies in the folder `super`.

## Requirements:
```
g++ >= 10.0
```

## Optional but strongly encouraged:
Although they are both supplied via the superbuild, it is strongly encouraged to use the package manager of your Linux distro to install these packages.
This is especially true for OpenCV, which can also be installed using `helper/build_opencv.sh` for Ubuntu 18.

Ubuntu 20 supplies both packages in a version that is recent enough.

```
OpenCV >= 4.2
Qt >= 5.10
```

## Building
To build the full project from the superbuild simply run the following commands:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../super
make
```
the binary of the HabiTrack-Gui should be compiled in `build/ui/gui/habitrack-gui`.


# GUI Explanation:
## Zooming:
control + wheel -> slow zooming
shift + control + wheel -> fast zooming

## Unary:
control + left click -> add unary
control + right click -> remove unary (meaning set back to computed unary)

## Unary-View:
green: "good" unary
yellow: "ok" unary
red: "not so good" unary
blue: "perfect" unary which is only used for manual unaries
this doesn't not mean that the tracking doesn't work here and gives a lot of false positives.
Usually only critical if there is a big continuous chunk of red unaries.

## Above Unary-View:
alternating colored "chunks" (darkgray and magenta)
If one chunk is optimized, the color is much lighter and there is a sound once a chunk (all all in the first run) is finished.

## Saving:
Optimize saves always
Manual unaries are only saved automatically, when optimized afterwards or when clicking File > Save.
Mike Super Button "saves" too, meaning clicking it again checks if files exists and loads them again (see special case "RAM full")

## Chunk Size:
Default 100, can be changed in preferences. This seems like a sensible default.

# FAQ:
## RAM full:
If program was killed when doing the full pipeline (with optimization). Try quitting and click Mike Super Button again.
This will just load the files computed in the first run (OpenCV memory issue we talked about).

If RAM is full (and or Swap too) in one of the early stages, try reducing the cache size to 200 in Tools > Preferences (not chunk size).

