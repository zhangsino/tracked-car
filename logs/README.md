# logs —— 实车日志

抓法：

```bash
source ~/.zshrc && get_idf          # 只是习惯，跑脚本其实不需要
python3 ~/r3car/logs/capture.py     # Ctrl-C 结束
```

会自动存成 `YYYY-MM-DD-HHMM.log`，每行前面带相对时间戳。

**别用 `idf.py monitor` 抓运行日志**：它会拉 DTR/RTS，偶尔把板子拉进下载模式，
而且日志不落盘。`capture.py` 开串口后立刻把 dtr/rts 拉低，只读不碰。

| 文件 | 是什么 |
|---|---|
| `2026-11-21-espnow-首跑.log.md` | ESP-NOW 版第一次跑通的原始日志（带解读） |
