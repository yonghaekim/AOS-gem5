# AOS gem5

## Dependencies
According to the [gem5 documentation](https://www.gem5.org/documentation/learning_gem5/part1/building/),
you will likely need the following packages.

In our environment, the version of scons often makes a trouble.

We recommend using scons-2.5.1 to avoid errors regarding python syntax errors.
```
build-essential git m4 scons zlib1g zlib1g-dev libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev python-dev python
```

## Clone Git Repositories
Clone the AOS-gem5 repository.
```
git clone https://github.com/yonghaekim/AOS-gem5.git .
```

## Build Instruction
```
scons ./build/ARM_MESI_Two_Level/gem5.opt -j8
```

## How to run SPEC2006 workloads?
We put an example script, `run_spec_from_scratch.sh`, under `$GEM5_PATH/exp_script`.

To enable AOS features, you will need programs instrumented using AOS compiler passes.

Please check out our [AOS-llvm](https://github.com/yonghaekim/AOS-llvm) repository for more details.

```
./run_spec_from_scratch.sh bzip2      # when running baseline programs
./run_spec_from_scratch.sh bzip2 AOS  # when running instrumented AOS programs
```

## Publications
```
@inproceedings{kim:aos,
  title        = {{Hardware-based Always-On Heap Memory Safety}},
  author       = {Yonghae Kim and Jaekyu Lee and Hyesoon Kim},
  booktitle    = {Proceedings of the 53rd IEEE/ACM International Symposium on Microarchitecture (MICRO)},
  month        = nov,
  year         = 2020,
  address      = {Athens, Greece},
}
```

## Original gem5 README

This is the gem5 simulator.

The main website can be found at http://www.gem5.org

A good starting point is http://www.gem5.org/about, and for
more information about building the simulator and getting started
please see http://www.gem5.org/documentation and
http://www.gem5.org/documentation/learning_gem5/introduction.

To build gem5, you will need the following software: g++ or clang,
Python (gem5 links in the Python interpreter), SCons, SWIG, zlib, m4,
and lastly protobuf if you want trace capture and playback
support. Please see http://www.gem5.org/documentation/general_docs/building
for more details concerning the minimum versions of the aforementioned tools.

Once you have all dependencies resolved, type 'scons
build/<ARCH>/gem5.opt' where ARCH is one of ARM, NULL, MIPS, POWER, SPARC,
or X86. This will build an optimized version of the gem5 binary (gem5.opt)
for the the specified architecture. See
http://www.gem5.org/documentation/general_docs/building for more details and
options.

The basic source release includes these subdirectories:
   - configs: example simulation configuration scripts
   - ext: less-common external packages needed to build gem5
   - src: source code of the gem5 simulator
   - system: source for some optional system software for simulated systems
   - tests: regression tests
   - util: useful utility programs and files

To run full-system simulations, you will need compiled system firmware
(console and PALcode for Alpha), kernel binaries and one or more disk
images.

If you have questions, please send mail to gem5-users@gem5.org

Enjoy using gem5 and please share your modifications and extensions.
