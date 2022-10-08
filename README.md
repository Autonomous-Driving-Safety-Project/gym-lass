# OpenAI Gym API for LASS

## Install Guide

1. System requirments: ubuntu 20.04
2. Install boost >= 1.71.0
3. Install cmake >= 3.16
3. Clone the code
```shell
git clone https://github.com/Autonomous-Driving-Safety-Project/gym-lass.git
cd gym-lass
git submodule init
git submodule update
```

4. Compile LASS
```shell
cd gym_lass/lass
mkdir build && cd build
cmake ..
cmake --build . --config Release -- -j8
```

5. Install as a python module
```shell
# in project root directory
pip install -e .
```

6. test
```shell
cd gym_lass/test
python test.py
```



## Acknowledgement

This project uses a part of [esmini](https://github.com/esmini/esmini).

