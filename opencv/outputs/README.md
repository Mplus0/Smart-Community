# 识别输出

从 Smart-Community 仓库根目录运行：

```bash
python -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json
```

默认结果为 `opencv/outputs/read/clear3/result.json` 和 `debug.png`。同名图片重复运行更新最近一次结果。

`python -m opencv demo` 的演示结果单独放在 `opencv/outputs/demo/`，不会覆盖实际读数目录。

JSON 记录原图位置、所用配置、UTC 处理时间、状态及读数。比较不同配置或同名照片时，使用 `--output` 指定不同目录。

所有生成的子目录都可删除，下次运行时自动创建。除了这份说明，输出内容均由仓库根目录的 Git 忽略规则排除。
