<!--
********************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Daniel Heß - initial API and implementation
********************************************************************************
-->
# sumo_if_ros
A catkin package, which enables exchange of information between ROS environment and [SUMO](http://eclipse.org/sumo).

## Getting Started
This module requires **docker** and **make** installed and configured for your user

### Building sumo_if_ros
1. clone the repository with recursive submodules
```bash
git clone --recurse-submodules -j8 <REPO>
```
or if you have already cloned the repository:
```bash
cd sumo_if_ros
git submodule update --init --recursive
```
2. run make
```bash
cd sumo_if_ros
make build
```

## External Libraries
sumo_if_ros depends on `sumo` that do not provide
distributions.  They are packaged and hosted for adore via docker.io.
All the build context for sumo is located in `sumo`. There is a provided
make file to build and publish sumo library. By default all sumo
submodule is disabled in the `.gitmodules` file. It has been previously 
published to docker.io. In order to build it you must first enable it 
 in the `.gitmodules` file. 

> **ℹ️ INFO:**
> The `sumo` submodue/library is disabled and will not be pulled. Enable them
> by modifying the `.gitmodules` and invoking 'git submodue update --init'.

> **ℹ️ INFO:**
> By default the sumo library not built. It is sourced first from local
> cache in /var/tmp/docker and second as pre-compiled docker image from docker.io.
The external cache for libsumo is not deleted or cleaned automatically. In order
to clean the libsumo cache located in `/var/tmp/docker` invoke the 
provided target:
```bash
make clean_sumo_library_cache
```
The sumo repository is big and it takes significant time to build.  In order to
save build time a pre-published build context is provided via docker.io.

It can be manually built by enabling the sumo submodule in the .gitmodules, 
updating the submodule, and manually invoking `make build_sumo`. This will build
sumo, as well as, libsumo.

### Linting
To lint the sumo_if_ros source code you can use the provide target:
```bash
make lint
```

### help target
To view useful targets you can run make help:
```bash 
make help
```
