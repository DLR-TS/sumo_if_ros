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
