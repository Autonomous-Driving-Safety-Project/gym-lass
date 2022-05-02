# OpenAI Gym API for LASS

## Install

1. 安装Boost >= 1.71.0

2. 安装cmake

3. 下载
```shell
git clone git@git.ustc.edu.cn:navigation_ng/envs/gym-lass.git
cd gym-lass
git submodule init
git submodule update
```

4. 编译LASS
```shell
cd gym_lass/lass
mkdir build && cd build
cmake ..
cmake --build . --config Release -- -j8
```

5. 作为python模块安装
```shell
# in project root directory
pip install -e .
```

6. 测试
```shell
cd gym_lass/test
python test.py
```

> 注: 此处尚且存在问题, 参见LASS/#1. 该问题只在开启可视化时出现, 不影响训练
