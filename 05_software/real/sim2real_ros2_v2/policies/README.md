# `v0.12.0` 策略文件

本阶段运行配置使用 `model_9600` 作为 Rough 策略，并保留 Crawl ONNX 和部署契约参考模型。

| 文件 | 用途 |
| --- | --- |
| `model_9600.onnx` | 里程计版本 Rough ONNX |
| `model_9600_fp16.engine` | 里程计版本 Rough TensorRT |
| `model_crawl.onnx` | Crawl 候选；当前配置使用 IK 后端 |
| `model_rough.onnx` | 原部署契约参考基线 |

其余 `model_6800`、`model_8400`、`model_10200` 等候选策略属于相邻实验或后续比赛版本，不在本 Tag 重复归档。
