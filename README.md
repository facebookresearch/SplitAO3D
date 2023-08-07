# PSAO: Point-Based Split Rendering for Ambient Occlusion
 [Thomas Neff](https://thomasneff.github.io/)\*<sup>1</sup>,
 [Brian Budge](https://scholar.google.com/citations?user=z67HnEwAAAAJ&hl=en)<sup>2</sup>,
 [Zhao Dong](http://flycooler.com/)<sup>2</sup>,
 [Dieter Schmalstieg](http://dieterschmalstieg.me/)<sup>1</sup>,
 [Markus Steinberger](https://www.markussteinberger.net/)<sup>1</sup>

 <sup>1</sup>Graz University of Technology, <sup>2</sup>Meta Reality Labs Research  
  \* The work was primarily done during an internship at Meta.
  
in Computer Graphics Forum Volume 42 (2023), Number 8

<img src='psao_teaser.png'/>

### Licensing
The majority of this project is licensed under TODO, except for adapted third-party code, which is available under separate license terms:

* Arcade test scene: NVIDIA CC-BY-NC-SA 3.0
* [Poisson sampling](https://github.com/cemyuksel/cyCodeBase/blob/master/LICENSE) is licensed under the MIT license 
* [FLIP](https://github.com/NVlabs/flip) is licensed under the BSD-3 license
* [zstd](https://github.com/facebook/zstd/blob/dev/LICENSE) is licensed under the BSD license
* [nanoflann](https://github.com/jlblancoc/nanoflann/blob/master/COPYING) is licensed under the BSD license
* [lz4](https://github.com/lz4/lz4/blob/dev/LICENSE) is licensed under the BSD license
* [argparse](https://github.com/p-ranav/argparse) is licensed under the MIT license

### General
This repository contains the source code for the paper "PSAO: Point-Based Split Rendering for Ambient Occlusion".
The codebase has been tested on Windows 11 using an RTX4090 GPU using Visual Studio 2019.

### Getting started

First clone our fork of Falcor 5.2 from [https://github.com/thomasneff/Falcor](https://github.com/thomasneff/Falcor).
```bash
git clone git@github.com:thomasneff/Falcor.git
cd Falcor
git submodule update --init --recursive
```

Make sure to follow the prerequesites mentioned in the Falcor README. 

Afterwards, go into the `Falcor/Source/Samples/` directory and clone this repository (`SplitAO3D`). 

```bash
cd Source/Samples
git clone https://github.com/facebookresearch/SplitAO3D
```

After that, use CMake to generate build files for Falcor.

You should now have a `FalcorServer` target in the `Samples` projects in your IDE (e.g., Visual Studio 2019). 
Set that as your startup target. 

Afterwards, for our test scenes to work, you need to create a symbolic link from `Falcor/Source/Samples/SplitAO3D/test_scenes` to `Falcor/media/test_scenes`.

By default, you should now be able to start the project and see the `arcade_with_animated_things` scene, that includes rigidbody and skinned animations and shows the result of PSAO with a low number of points by default.

In `SplitAO3D/point_ao_split_rendering/ServerMain.cpp` you can find additional command line options that were used for the evaluation of the paper. 
The main renderer is implemented in `ServerPointRenderer.h/.cpp`.

### Citation

If you find this repository useful in any way or use/modify PSAO in your research, please consider citing our paper:


```bibtex
@article{neff2023psao,
  year = { 2023 },
  eprint = { https://onlinelibrary.wiley.com/doi/pdf/10.1111/cgf.14864 },
  url = { https://onlinelibrary.wiley.com/doi/abs/10.1111/cgf.14864 },
  doi = { https://doi.org/10.1111/cgf.14864 },
  keywords = { CCS Concepts, • Computing methodologies → Rendering },
  number = { 8 },
  volume = { 42 },
  journal = { Computer Graphics Forum },
  title = { {PSAO}: Point-Based Split Rendering for Ambient Occlusion },
  author = {Neff, T. and Budge, B. and Dong, Z. and Schmalstieg, D. and Steinberger, M.},
}
```

