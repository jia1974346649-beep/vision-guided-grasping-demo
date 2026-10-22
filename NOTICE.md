# 来源、匿名化与第三方说明

## 本仓库

本仓库是作者与 AI 协作完成的独立匿名演示。公开内容为重新编写的通用实现、合成桌面场景、安装说明，以及由当前演示新生成的运行结果。

公开范围不包含原项目的公司身份、客户资料、内部业务文档、设备配置、个人联系人、实验归档、训练权重或原始提交历史。作者与 AI 的实际分工见 [贡献说明](docs/contribution.md)。

本仓库新编写的演示代码和文档采用根目录 [LICENSE](LICENSE) 中的 MIT 许可。该许可不改变第三方依赖、机器人描述文件或网格的归属。

## 第三方基础与资源

| 来源 | 使用方式 | 归属与许可说明 |
| --- | --- | --- |
| [PyBullet / Bullet](https://github.com/bulletphysics/bullet3) | 通过 `requirements.txt` 安装，用于物理仿真、渲染与内置 IK | Bullet 主仓库通用代码使用 [zlib 许可](https://github.com/bulletphysics/bullet3/blob/master/LICENSE.txt)，特定第三方资源遵循各自许可 |
| [Panda 模型资源](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data/franka_panda) | 从已安装的 `pybullet_data` 加载 URDF 和网格，本仓库不另行复制模型文件 | 该资源目录提供 [Apache 2.0 许可](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_data/franka_panda/LICENSE.txt)；模型不是本仓库作者制作 |
| [NumPy](https://github.com/numpy/numpy) | 数值运算 | 依赖包自带许可适用 |
| [Pillow](https://github.com/python-pillow/Pillow) | GIF、PNG 和报告图像输出 | 依赖包自带许可适用 |

动画和截图展示的是使用上述第三方机器人模型生成的合成仿真画面。仓库名称、说明与 MIT 许可不应被理解为对 Panda 模型、PyBullet 引擎或其品牌的所有权声明。

安装依赖时请保留随依赖提供的许可与版权说明。若后续改为直接分发第三方模型文件，应同时遵守其完整的再分发条件。
