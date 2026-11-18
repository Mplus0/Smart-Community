# Windows 单目温湿度表读数

适用于得力 No.9011。温度和湿度分别标定、寻针和取整，运行时只需 Python、OpenCV 和 NumPy。以下 PowerShell 命令从仓库根目录运行。

## 安装与单图检查

```powershell
py -3 -m venv opencv/.venv
opencv/.venv/Scripts/python.exe -m pip install -r opencv/requirements.txt
opencv/.venv/Scripts/python.exe opencv/meter.py read opencv/example/face.jpg --overlay opencv/outputs/check.jpg
```

内置模板裁自需求照片，刻线与轴心为人工标注，适合先检查流程。使用自己的原始照片时，将命令中的 face.jpg 路径换成照片路径。程序输出 JSON；temperature 单位为 °C，humidity 单位为 %RH。紫色线段表示选中的指针方向。alignment_failed 表示配准失败，aligned 表示完成表面配准，各字段为 null 时表示对应指针未可靠读出。

## 标定实测仪表

用实际相机正对表面拍清晰照片，裁到完整表盘，尽量少留背景。新建标定可在相机、型号或模板质量变化时使用。

用 UGREEN 相机（编号 1）拍摄 4K 标定照时，先关闭相机识别窗口，再运行以下命令。在预览中等到刻线清晰后按 S，程序保存 3840×2160 原始帧。将标定结果保存到新目录，便于与已有标定对比。

```powershell
opencv/.venv/Scripts/python.exe opencv/meter.py capture opencv/outputs/face-4k.png --index 1
opencv/.venv/Scripts/python.exe opencv/meter.py calibrate opencv/outputs/face-4k.png opencv/local-profile-4k
```

已有正面照片时，可直接运行标定。

```powershell
opencv/.venv/Scripts/python.exe opencv/meter.py calibrate .\正面表盘.jpg opencv/local-profile
```

按窗口提示先点温度针的轴心，再依次点 −30、−20 至 50 的长刻线中点，回车确认。接着点湿度针的轴心，再点 0、10 至 100 的长刻线中点并回车。U 撤销，Esc 取消。输出目录保存模板图和 profile.json。标定后可给 read 或 camera 命令加上 --profile opencv/local-profile/profile.json。

## 相机实测

当前验收使用 UGREEN 相机（编号 1）和 `opencv/local-profile-4k/profile.json`。配套模板是同目录的 `face.png`。

```powershell
opencv/.venv/Scripts/python.exe opencv/meter.py camera --index 1 --profile opencv/local-profile-4k/profile.json --resolution 4k
```

程序通过 Windows DirectShow 打开相机，默认申请 1280×720。可加 `--resolution 1080p` 或 `--resolution 4k` 比较画质，实际收到的尺寸显示在窗口左上角。按 Q 或 Esc 退出。窗口持续显示相机原始画面，逐帧读取指针并将紫线投影回原图。定位时将长边限制在 1400 像素以内，透视校正和读数使用原始帧。模板匹配短暂失败时，可在最后一次成功匹配后的 2 秒内用相邻帧光流跟踪表盘特征，窗口显示 tracking。无法确认表盘位置时显示 alignment_failed，并清空读数和紫线。相机编号根据设备调整。

## 实现和验收流程

1. 每帧用 SIFT 匹配表面特征，使用 RANSAC 求单应矩阵。对匹配内点数量、比例、覆盖范围和表盘投影形状进行检查，通过后在模板坐标中读数。
2. 两个表盘各自从轴心沿有效量程搜索暗线，比较针身与两侧的亮度，并检查内外两段连续性。最强方向对比太低或有接近的竞争方向时返回空值。
3. 用相邻长刻线做角度插值。输出时温度向下取到 1 °C 刻度，例如 20.9 °C 取 20 °C、−17.3 °C 取 −18 °C。湿度向下取到 2 %RH 刻度，输出 0、2、4 至 100，例如 79.9 %RH 取 78 %RH。
4. 在真实停车距离拍摄独立样本，覆盖正面、轻微斜拍、偏暗、反光和不同针位。逐图记录人工值、程序值及叠加图，对两根针分别统计完全一致率、误差不超过 1 的比例和未读出率。应单列失败帧，以免高拒读率掩盖误差。
5. Windows 实测确认后再接入实车工作空间。模板照片及其变换只能用于软件回归，无法验证物理视差、真实光照或相机帧率。
