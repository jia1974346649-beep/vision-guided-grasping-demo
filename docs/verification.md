# 发布验证记录

记录日期：2026-10-22。这里的结果来自本次独立匿名演示，没有沿用原项目论文、历史图表或训练结果。

## 实际环境

| 项目 | 本次记录 |
| --- | --- |
| 仿真环境 | Windows 主机中的 WSL2，Debian GNU/Linux，Python 3.9.2 |
| 安装 | 在新建虚拟环境中，从官方 PyPI 安装固定版本的 Linux 二进制包 |
| 依赖 | NumPy 1.26.4、PyBullet 3.2.7、Pillow 11.3.0 |
| 渲染与控制 | `DIRECT` + TinyRenderer，固定物理步长 1/240 秒，无 GPU、真实相机或机器人 |
| 浏览器 | Chromium 155.0.8059.85；报告为本地静态 HTML |
| 额外发布审计 | Windows / Python 3.12.14，仅用于文件、媒体和 Git 检查，没有在该环境运行 PyBullet |
| 独立 CI 环境 | GitHub Actions 的 Ubuntu runner，Python 3.11.17，重新安装固定二进制依赖 |

## 测试与演示

| 检查 | 实际结果 |
| --- | --- |
| 控制与视觉 | 28 项通过，涵盖 PID 饱和与积分恢复、DLS 奇异/矩形矩阵、相机反投影和分割边界 |
| 仿真集成 | 8 项通过；实际验证提升与保持、种子复现、预期失败、录帧和异常后断开物理连接 |
| 报告与 CLI | 8 项通过；检查指标与轨迹一致、媒体解码、HTML 转义、无效结果不生成报告及退出码 |
| 发布审计测试 | Windows 中 24 项全部通过；Linux 中 22 项通过、2 项因未安装 Git 跳过，这 2 项已在 Windows 通过 |
| 测试总口径 | 共 68 项；独立 CI 中全部通过、0 跳过。此前本地 Linux 为 66 通过、2 跳过，Windows 补验这 2 项，分别保留记录 |
| 独立 CI 复现 | 提交 `51e62c8d1746800957b58df0816ec580f4230aa4` 的固定依赖安装、发布快照扫描、68 项测试及三个 CLI 场景均通过，见下方运行链接 |
| 干净源码复现 | 按清单将 32 个文件打包、解压至新目录并逐项核对哈希；复用上述已安装虚拟环境，三个 CLI 场景均实际运行并生成完整五个文件，源码运行前后未变 |
| 固定种子回归 | `7、11、23、41、97` 五个合成场景均达成当前辅助夹持判定；两个负向场景均返回预期失败 |
| 浏览器报告 | 两个图片资源可解码，指标 JSON 链接与本次 seed 7 结果一致；1440、768、375、320 CSS 像素宽度及展开完整指标后均无横向溢出，重载后的检查未记录页面或控制台错误 |
| 媒体 | 阶段图与 58 帧 GIF 均由本次仿真生成；人工查看阶段图及 GIF 代表帧，自动检查全部帧可解码、元数据及尾随内容 |

Linux 回归使用：

```bash
.venv/bin/python -B -W error::ResourceWarning -m unittest discover -s tests -v
.venv/bin/python scripts/run_regression.py --output runs/regression.json
```

[已核对的独立 CI 运行](https://github.com/jia1974346649-beep/vision-guided-grasping-demo/actions/runs/39719645168)使用与本次样例相同的仿真代码，完整日志记录了 68 项通过、0 跳过，以及正常场景成功和两个场景的预期失败。[工作流配置](../.github/workflows/demo.yml)与[最新状态](https://github.com/jia1974346649-beep/vision-guided-grasping-demo/actions/workflows/demo.yml)可公开查看。

## 本次样例数据

[完整回归记录](../examples/regression.json)保留了五个种子的指标和两种失败原因。seed 7 的[原始指标](../examples/seed-7/report.json)与[轨迹 CSV](../examples/seed-7/trajectory.csv)对应同一次运行：

| 指标 | 记录 |
| --- | --- |
| 夹持方式 | `constraint-assisted; not contact-only` |
| 物理步数 | 619 |
| 仿真时间 | 2.579167 秒，来自物理步数，不是机器运行耗时 |
| 球心估计误差 | 约 1.833 mm |
| 相对初始提升高度 | 约 221.341 mm |
| 保持要求 | 0.5 个仿真秒内，始终高于初始位置 120 mm |
| 创建固定约束的瞬间位移 | 0，不通过重置位置瞬移物体 |
| 负向场景 | `red_target_not_detected`、`estimated_target_out_of_workspace` |

这些是受限工作区、已知球半径、顶置相机与辅助附着条件下的小样本回归，不是通用抓取成功率或实机性能数据。真值用于误差评估和显式辅助附着，运动目标由视觉估计提供。

## 公开文件检查

`publication-manifest.json` 明确列出可公开文件及 SHA-256。公司与个人标识的私有关键词列表放在仓库外，不随项目发布。

```bash
.venv/bin/python scripts/audit_public_package.py --root . --git-history
```

检查覆盖文件名、文本、精确媒体哈希、PNG/GIF 元数据与结构，以及可达 Git 历史和作者信息；图像像素另作人工复核。检查仅支持已审查的快照，不构成绝对匿名保证。

原项目论文、身份材料、训练数据、模型权重、旧实验输出、依赖环境与原始 Git 历史均不纳入公开包。机器人模型来自第三方依赖，来源和许可见 [NOTICE](../NOTICE.md)。

当前没有实机、自然接触摩擦抓取、强化学习训练或泛化能力验证。Windows 原生与 macOS 安装路线也未计入实测。
