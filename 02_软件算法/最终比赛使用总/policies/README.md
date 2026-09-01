# 比赛部署策略

这里只保留最终运行配置实际使用的 Rough 和 Wall 策略。Crawl 在比赛配置中使用 IK 后端，因此不归档未使用的 Crawl RL engine。

| 用途 | 文件 | SHA-256 |
| --- | --- | --- |
| Rough ONNX | `model_6800.onnx` | `3C994BDD3434AD15770A52AC0E8D229F502F00D6511CDD42C2E2C742301AEF13` |
| Rough TensorRT | `model_6800_fp16.engine` | `BDC6583AFD594E84D9A4AAA4BB2EEE279C7EBF207A57CF4802A2DF5309C5F663` |
| Wall ONNX | `model_84.onnx` | `E3A447782DF6C6E11E66C3ACBE41697FF88EBF5E75DCAB65EFCF9A2EB1CA639F` |
| Wall TensorRT | `model_84_fp16.engine` | `0FE2001113B7146B589D7A62A6E54BAD68574765EBA45782FB41A26F65A8721D` |

`model_6800.onnx` 与 `05_software/train/rc_mjlab/model_6800.onnx` 哈希一致。此处重复保留是为了让真机工作区可以独立部署。

TensorRT 文件只作为比赛机历史产物；更换 JetPack、TensorRT 或 GPU 后应从 ONNX 重新构建并重新核对数值误差。
