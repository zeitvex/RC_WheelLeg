# 8DOF 串联足机械资料

`CAD/` 保存通过中期检查的 8DOF 串联足机器人 SolidWorks 源文件。

## 主要文件

- `DOG总装.SLDASM`：整机总装候选入口
- `LDOG身体.SLDASM`：机身装配体
- `单一腿部.SLDASM`：单腿装配体
- `动力单元.SLDASM`：腿部动力单元
- `小腿.SLDASM`：小腿装配体
- `*.SLDPRT`：机身、腿部、同步带传动和电机连接零件

## 使用建议

1. 保持 `CAD/` 内文件的相对位置和文件名不变。
2. 优先从 `CAD/DOG总装.SLDASM` 打开整机。
3. 若出现引用丢失，使用 SolidWorks 的查找引用功能指向 `CAD/`。

当前资料尚未包含 8DOF STEP 总装和二维 PDF 工程图。
- 提示：SolidWorks版本可能需要大于等于SolidWorks2026
