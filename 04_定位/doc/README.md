# 定位

这一类内容负责 Odin 的扫图、建图和重定位。

## Odin 在这里做什么

- 提供里程计和传感器输入
- 参与 SLAM 建图
- 在已有地图里完成重定位
- 为 `map`、`odom`、`base_link` 建立关系

## 这次移进来的文件

- `Odin1扫图与重定位总览.md`
- `Odin1工作空间说明.md`
- `Odin1重定位指南.md`
- `Odin1使用手册.md`

## 相关子目录

- `D:\git\odin\odin_open\04_定位\地图与重定位`

## 主要调用链

```text
Odin sensor / odom -> driver -> map save / relocalization -> map -> odom -> base_link
```
