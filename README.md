<html>
<h1 align="center">Rust Delaunay Triangulation</h1>
<p align="center" >

<img src="https://img.shields.io/badge/language-Rust-blue.svg" />

<img src="https://img.shields.io/github/license/nelsonatgithub/nlsn-delaunay-refine" />

<img src="https://img.shields.io/github/issues/nelsonatgithub/nlsn-delaunay-refine" />

<img src="https://img.shields.io/github/stars/nelsonatgithub/nlsn-delaunay-refine" />

<img src="https://img.shields.io/github/forks/nelsonatgithub/nlsn-delaunay-refine" />

</p>
<p align="center" >

<img src="https://travis-ci.org/nelsonatgithub/nlsn-delaunay-refine.svg?branch=dev" />

<a href="https://codecov.io/gh/nelsonatgithub/nlsn-delaunay-refine">
  <img src="https://codecov.io/gh/nelsonatgithub/nlsn-delaunay-refine/branch/dev/graph/badge.svg" />
</a>

</p>

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
    - [x] publishing release to crates.io
    - [x] 2D Delaunay Refinement
    - [ ] review User Programming Interface

    - [ ] 3D Delaunay Triangulation
    - [ ] 3D Delaunay Refinement

# Features

- Incremental Vertex Insertion
- Segment Constraints
- Holes
- Boudanry
- Refinement
- Tetrahedralization (*in progress*)

# API

> In progress

# Contributions

At first, clone the repository, with a cargo environment. Fork it if you want. Run the tests. Read the code.

Open an issue with suggestions, code reviews, refactoring.

# References

1. Cheng, Siu-Wing; Dey, Tama Krishna; Shewchuk, Jonathan Richard. Delaunay Mesh Generation. 2013 by Taylor & Francis Group, LLC.

