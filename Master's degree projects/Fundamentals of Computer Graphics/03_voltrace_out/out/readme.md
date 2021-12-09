# Yocto/Pathtrace: Tiny Volumetric Path Tracer

In this homework, you will learn how to build a simple path tracer with support
for subdivision surfaces, displacement and subsurface scattering.
In particular, you will learn how to

- handle subdivision surfaces,
- handle normal mapping (we have implemented displacement for you),
- write a path tracer with support for homogeneous volumes.

## Framework

The code uses the library [Yocto/GL](https://github.com/xelatihy/yocto-gl),
that is included in this project in the directory `yocto`.
We suggest to consult the documentation for the library that you can find
at the beginning of the header files. Also, since the library is getting improved
during the duration of the course, se suggest that you star it and watch it
on Github, so that you can notified as improvements are made.

In order to compile the code, you have to install
[Xcode](https://apps.apple.com/it/app/xcode/id497799835?mt=12)
on OsX, [Visual Studio 2019](https://visualstudio.microsoft.com/it/vs/) on Windows,
or a modern version of gcc or clang on Linux,
together with the tools [cmake](www.cmake.org) and [ninja](https://ninja-build.org).
The script `scripts/build.sh` will perform a simple build on OsX.
As discussed in class, we prefer to use
[Visual Studio Code](https://code.visualstudio.com), with
[C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) and
[CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
extensions, that we have configured to use for this course.

You will write your code in the file `yocto_pathtrace.cpp` for functions that
are declared in `yocto_pathtrace.h`. Your renderer is called by `yscenetrace.cpp`
for a command-line interface and `ysceneitraces.cpp` that show a simple
user interface.

This repository also contains tests that are executed from the command line
as shown in `run.sh`. The rendered images are saved in the `out/` directory.
The results should match the ones in the directory `check/`.

## Functionality

In this homework you will implement the following features:

- **Subdivision Surfaces** in functions `subdivide_catmullclark()`:
  - implement Catmull-Clark subdivision following the slides
  - while the solution should match the one in Yocto/Shape, your
    implementation _cannot_ use the Yocto one, neither calling it nor
    adapting the code – I will be able to tell since the Yocto/Shape code
    supports more features
- **Normal Mapping** in function `eval_normalmap()`:
  - implement normal mapping as per the slides
  - you can use `triangle_tangents_fromuv()` to get tangents
- **Volumetric Path Tracing** in function `trace_volpath()`:
  - follow the slides and implement a volumetric path tracer
  - you have to also implement all support functions,
    namely `XXX_transmittance()` and `XXX_scattering()`
  - you can use the corresponding functions in Yocto/Math

## Extra Credit

As usual, here are the extra credit work you can choose to do. As before,
the maximum points sum is the one reported above. But you can do more of these
if desired.

New this time is that if you do extra credit, you have to **prepare a short PDF**
that describes which feature you have implemented and includes images for
those features. **Without the PDF the extra credit will not be graded**.
The PDF should be vrey short and just say what you did succinctly, like with
a bullet points lists.

- **Adaptive rendering**:
  - implement a stropping criterion to focus render resources when more needed
  - technique is described [here](https://jo.dreggn.org/home/2009_stopping.pdf)
  - possible implementation [here](https://github.com/mkanada/yocto-gl)
  - your job here is to integrate really well the code, provide test cases and make it run interactively
