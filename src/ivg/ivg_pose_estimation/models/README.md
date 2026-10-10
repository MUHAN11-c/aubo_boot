# rembg u2net 权重

`u2net.onnx` 约 168MB，超过 Gitee 社区版单文件配额与 GitHub 100MB，**不随 git 入库**。运行时 `U2NET_HOME` 指向本目录；本机已有该文件则保留。

clone 后若本目录没有 `u2net.onnx`：

```bash
./fetch_u2net.sh
```

也可在首次抠图时由 rembg/pooch 自动下载（需能访问 GitHub）。官方地址与 rembg `v0.0.0` 发布物一致：

https://github.com/danielgatis/rembg/releases/download/v0.0.0/u2net.onnx
