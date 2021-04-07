Zooming:
control + wheel -> slow zooming
shift + control + wheel -> fast zooming

Unary:
control + left click -> add unary
control + right click -> remove unary (meaning set back to computed unary)

Mike Super Button:
combines Feature Extraction, Matching (+ Transformations), Unary Extraction
and NOT Unary Optimization

Unary-View:
green: "good" unary
yellow: "ok" unary
red: "not so good" unary
blue: "perfect" unary which is only used for manual unaries
this doesn't not mean that the tracking doesn't work here and gives a lot of false positives.
Usually only critical if there is a big continuous chunk of red unaries.

Above Unary-View:
alternating colored "chunks" (darkgray and magenta)
If one chunk is optimized, the color is much lighter and there is a sound once a chunk (all all in the first run) is finished.

Saving:
Optimize saves always
Manual unaries are only saved automatically, when optimized afterwards or when clicking File > Save.
Mike Super Button "saves" too, meaning clicking it again checks if files exists and loads them again (see special case "RAM full")

Chunk Size:
Default 100, can be changed in preferences. This seems like a sensible default.

RAM full:
If program was killed when doing the full pipeline (with optimization). Try quitting and click Mike Super Button again.
This will just load the files computed in the first run (OpenCV memory issue we talked about).

If RAM is full (and or Swap too) in one of the early stages, try reducing the cache size to 200 in Tools > Preferences (not chunk size).

needed packages when installing qt5 from source:
libx11-dev
libx11-xcb-dev
libxext-dev
libxfixes-dev
libxi-dev
libxrender-dev
libxcb1-dev
libxcb-glx0-dev
libxcb-keysyms1-dev
libxcb-image0-dev
libxcb-shm0-dev
libxcb-icccm4-dev
libxcb-sync0-dev
libxcb-xfixes0-dev
libxcb-shape0-dev
libxcb-randr0-dev
libxcb-render-util0-dev
libxcd-xinerama-dev (should be libxcb-xinerama0-dev on ubuntu 18.04)
libxkbcommon-dev
libxkbcommon-x11-dev
