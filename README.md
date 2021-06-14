# modularslam

## Necessary packages

### SFML
```
sudo apt-get install libsfml-dev
```

### GTSAM
Add PPA
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
```
Install:
```
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Installation
```bash
mkdir build
cd build/
cmake -DBUILD_SHARED_LIBS=ON
sudo make install
```