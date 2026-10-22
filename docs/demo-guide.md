# 演示使用指南

## 1. 安装环境

按 [README](../README.md#安装与运行) 准备 Linux x86-64 或 Windows WSL2，并创建独立虚拟环境。推荐 Python 3.10 / 3.11；Windows 原生、macOS 和 Python 3.12 的 PyBullet 安装通常需要源码编译，当前没有验证。所有下列命令在 Linux / WSL2 内、从仓库根目录执行。

Windows PowerShell 用户先运行 `wsl --list --verbose`，再用 `wsl -d Ubuntu` 进入已安装的发行版；`Ubuntu` 需替换为自己的发行版名称。无需为运行本演示配置 GPU。

```bash
.venv/bin/python --version
.venv/bin/python -m pip install --only-binary=:all: -r requirements.txt
```

## 2. 跑一次完整流程

```bash
.venv/bin/python -m grasp_demo --seed 7 --case nominal --output runs/demo
```

程序在后台运行真实物理仿真，不会打开机器人操作窗口。等待命令结束后，用浏览器打开 `runs/demo/report.html`；WSL2 用户可运行 `explorer.exe .`，再从 Windows 文件管理器打开报告。报告可以离线查看，运行速度受 CPU 性能影响。

查看报告时按以下顺序理解结果：

1. 确认场景、随机种子和 `constraint-assisted` 夹持模式。
2. 查看目标检测、估计球心与定位误差。
3. 观察接近、闭合和抬升阶段。
4. 对照成功状态、抬升高度与失败原因。
5. 需要进一步检查时打开 `trajectory.csv` 与 `report.json`。

网页、GIF 和 PNG 都来自本次运行，修改种子或场景后应使用新的输出目录，便于保留对照证据。

## 3. 查看可解释的失败

```bash
.venv/bin/python -m grasp_demo --seed 7 --case no-target --output runs/no-target
.venv/bin/python -m grasp_demo --seed 7 --case out-of-workspace --output runs/out-of-workspace
```

这两类场景的预期结果是流程拒绝继续动作并保留原因。它们用于展示错误处理，不计为正常抓取成功。

## 4. 更改随机种子

```bash
.venv/bin/python -m grasp_demo --seed 11 --case nominal --output runs/seed-11
```

随机种子改变受控范围内的场景布局。若出现失败，应先查看定位误差、阶段事件与轨迹，而不是只更换种子隐藏失败。不同平台的具体数值可能略有差别。

## 5. 执行测试

```bash
.venv/bin/python -m unittest discover -s tests -v
```

测试结果与完整仿真报告回答不同的问题：前者检查具体函数和契约，后者展示系统在选定场景中的实际行为。发布前的实际检查见[验证记录](verification.md)。

仓库的 [GitHub Actions](https://github.com/jia1974346649-beep/vision-guided-grasping-demo/actions/workflows/demo.yml) 会在 Linux / Python 3.11 中执行固定依赖安装、测试及三个场景的 CLI 运行。打开某次运行可查看各步骤的实际结果；CI 不上传用户数据或模型权重。

## 常见问题

**没有动画窗口。**这是 `DIRECT` 离屏模式的正常行为。请在命令结束后打开输出报告。

**提示缺少 `pybullet`、`numpy` 或 `PIL`。**通常是安装与运行使用了不同的解释器。用同一个虚拟环境 Python 执行 `-m pip install -r requirements.txt` 和 `-m grasp_demo`。

**提示没有匹配的预编译包。**确认是在 Linux x86-64 / WSL2 内使用 Python 3.9、3.10 或 3.11。固定的 PyBullet 版本没有 Windows、macOS 或 CPython 3.12 wheel，安装路线见 [README](../README.md#安装与运行)。`--only-binary=:all:` 会直接报告不匹配，避免误以为编译卡死。

**PowerShell 不允许运行激活脚本。**推荐路线在 WSL2 中直接调用 `.venv/bin/python`，不运行 PowerShell 激活脚本，也不需要更改 Windows 执行策略。

**成功了是否说明可以上真实机器人？**当前结果说明受控仿真与约束辅助夹持的完整链路能够运行。真实机器人还需要标定、通信、限位、碰撞与力控等独立验证。
