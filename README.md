# Rust Delaunay Triangulation

<html>
<img src="https://img.shields.io/github/tag/expressjs/express.svg" />

<img src="https://img.shields.io/github/languages/top/badges/shields.svg" />

<img src="https://img.shields.io/github/issues/badges/shields.svg" />

<img src="https://img.shields.io/github/license/mashape/apistatus.svg" />

</html>

# Description

This repository implements Delaunay Triangulation in Rust, according to this reference [1].

The major objective is to implement a 3D refinement in pure Rust, so that it may be portable to wasm-pack applications.

I've searched for some packages in open repositories. There were good jobs and efficient implementations ([svew](https://github.com/svew/rust-voroni-diagram), [mourner](https://github.com/mourner/delaunator-rs), [tynril](https://github.com/tynril/rtriangulate), [ucarion](https://github.com/ucarion/voronoi-rs), [d-dorazio](https://github.com/d-dorazio/delaunay-mesh)), but none are extensible to this purpose. Some lack documentation, some follow other approaches.

# Approach

The approach is to implement `Bowyer Watson` incremental insertion algorithm, with `ghost triangles` and `conflict graph` . This approach is extensible to 3D, given the proper handle to sliver exudation and smooth surfaces.

The choice for `Rust` is due to its portability in sereral rust contexts and its integration to `Javascript` through `wasm-pack` .

# Task List

    - [x] 2D Delaunay Triangulation
    - [ ] publishing release to crates.io
    - [ ] 2D Delaunay Refinement

    - [ ] 3D Delaunay Triangulation
    - [ ] 3D Delaunay Refinement

# Development

This repository is build with Rust and Cargo.

``` bash
# Run tests
> cargo test

# Build the module
> cargo build
```

# References

1. Cheng, Siu-Wing; Dey, Tama Krishna; Shewchuk, Jonathan Richard. Delaunay Mesh Generation. 2013 by Taylor & Francis Group, LLC.

