# h264dec 对标 FFmpeg 9.0.2：功能、性能与差距分析

日期：2026-11-02  
对象：`H:\mycode\h264dec\h264dec`（8-bit 4:2:0，10-bit 4:0:0 / 4:2:0 / 4:2:2 / 4:4:4 H.264 解码库）  
参考：FFmpeg 9.0.2（MSYS2 构建，`ffmpeg.exe` / `ffprobe.exe`，x86-64）  
测试机：Intel Core i7-12700K，8 物理核 / 16 逻辑核，Windows，GCC 16.2.0

---

## 0. 结论摘要

**一句话：这不是一个“全面落后”或“全面超越”的关系，要分两个层面看。**

| 层面 | 结果 |
| --- | --- |
| 纯解码器吞吐（丢弃图像） | 8-bit：单线程落后 FFmpeg **1.15x–1.27x**，8–12 线程落后 **1.30x–1.72x**；10-bit 4:2:0：1 线程 1.52x，12 线程 1.20x；10-bit 4:4:4：1.56x / 1.14x；10-bit 4:2:2：1.49x / 1.06x（720p） |
| 文件 → YUV 的实际作业 | 8–12 线程**反超 FFmpeg 25%–37%**（720p/1080p），6 线程约持平 |
| 功能覆盖 | 8-bit 4:2:0 的 Baseline/Main/High/High 10 与 10-bit 4:0:0/4:2:0/4:2:2/4:4:4（High 10、High 4:2:2、High 4:4:4 Predictive）覆盖完整且逐字节一致；**8-bit 4:2:2/4:4:4、12-bit、FMO、数据分区、硬件加速缺失** |
| 稳定性/正确性 | 27/27 语料逐字节一致；29 组差分 sweep 一致；fuzz 324 变体无崩溃；soak/claim_hammer 通过；sanitizer 有项目记录 |
| 内存 | 1080p 峰值 RSS 比 FFmpeg 低 **8%–33%** |

因此：

- 如果目标是“播放器/转码器里把文件解码成 YUV”，**当前实现已经能在 8 线程以上超过 FFmpeg**，主要赢在输出路径和多线程吞吐；
- 如果目标是“作为纯 H.264 解码内核在所有线程数上超过 FFmpeg”，**还没有做到**，单线程差距约 20%，多线程差距约 30%–70%；
- 如果目标是“功能全面对标 FFmpeg”，**10-bit 已补齐（4:0:0/4:2:0/4:2:2/4:4:4）并通过 1/8/12 线程逐字节验证**；剩下的最大缺口是 8-bit 4:2:2/4:4:4、12-bit 和硬件加速。10-bit 目前走标量 16-bit 像素路径，功能正确但速度仍明显落后 FFmpeg。

---

## 1. 测试环境与口径

| 项 | 值 |
| --- | --- |
| CPU | i7-12700K，8P+0E（Windows 报告 8 核 16 线程） |
| OS | Windows，MSYS2 / MinGW-w64 |
| 编译器 | GCC 16.2.0，`-O3 -g -std=c99` |
| 参考 | FFmpeg 9.0.2，MSYS2 官方构建 |
| 720p 片源 | `testdata/bench/long_720p.h264`，1500 帧，High |
| 1080p 片源 | `testdata/bench/long_1080p.h264`，1000 帧，High |
| 隔行片源 | `testdata/bench/long_paff_720p.h264`，1000 个编码图像（415 个场图像），High |
| 吞吐口径 | 两边都丢弃图像（`h264dec_example ... -` 对 `ffmpeg -f null -`） |
| 端到端口径 | 两边都写 YUV 到空设备（`... NUL` 对 `-f rawvideo -y NUL`） |
| 统计 | 每个配置 3 次，取中位数；两边交替运行，同一时间窗口 |

绝对 fps 会随桌面负载变化，**看比值和缩放曲线，不要看单次绝对值**。

复现：

```sh
# 纯解码
REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh

# 文件 -> YUV
SINK=1 REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh

# 换片源
CLIP=../testdata/bench/long_1080p.h264 FRAMES=1000 ...
```

---

## 2. 功能对标

### 2.1 已支持并且验证过的能力

| 能力 | 状态 | 验证方式 |
| --- | --- | --- |
| Baseline / Main / High，8-bit 4:2:0 | 支持 | 27 流逐字节对 FFmpeg |
| CAVLC / CABAC | 支持 | 语料 + 29 组 sweep |
| 4x4 / 8x8 变换、缩放矩阵、8-bit lossless | 支持 | `high_8x8`、`high_cqm`、`high_lossless` |
| MBAFF / PAFF、场加权预测、Pic-Aff direct | 支持 | 5 个隔行流 + 差分 sweep |
| 加权预测（显式/隐式）、B-pyramid | 支持 | `weighted_*`、`b_*`、sweep |
| 长期参考帧、MMCO 1–6、POC type 0/1/2 | 支持 | 语料 + sweep |
| 多 slice、任意 slice 顺序、slice 长度限制 | 支持 | `multi_cavlc`、`slice_max_size`、sweep |
| `constrained_intra_pred`、deblocking 约束 | 支持 | `high_constrained`、`high_nodeblock` |
| 裁剪（奇数尺寸）、`max_num_reorder_frames` | 支持 | `high_odd_crop`、输出顺序测试 |
| SEI：pic timing、recovery point、encoder user data | 支持 | `pic_timing`、`sei_check` |
| Annex-B / 1/2/4 字节长度前缀（AVCC） | 支持 | `tests/avcc.sh` |
| 损坏码流不崩溃、继续解码、错误标记 | 支持 | fuzz 324 变体、stability |
| 帧级多线程 + 可抢占的 loop filter 队列 | 支持 | selfcheck 0/2/3/6/8/10/12 槽逐字节一致 |

本次重新执行了 `make check`（含 `compare.sh` 27/27、AVCC、SEI、API、soak、claim_hammer），全部通过。

### 2.2 缺失或受限的能力

| 能力 | FFmpeg | h264dec | 影响 |
| --- | --- | --- | --- |
| High 10（10-bit 4:2:0） | 支持 | **支持** | 已逐字节验证；当前为标量 16-bit 像素路径，性能待 SIMD 优化 |
| High 4:2:2（10-bit 4:2:2） | 支持 | **支持** | 10-bit 4:2:2 已逐字节验证；8-bit 4:2:2 仍未接受 |
| High 4:4:4 Predictive（10-bit 4:4:4） | 支持 | **支持** | 10-bit 4:4:4（含无损 transform bypass）已逐字节验证；8-bit 4:4:4 仍未接受 |
| Monochrome 4:0:0（10-bit） | 支持 | **支持** | 10-bit 4:0:0 已逐字节验证；8-bit 4:0:0 仍未接受 |
| 12-bit | 支持 | 不支持 | 更少见 |
| FMO / slice groups | 支持 | 不支持（解析时报错） | 老式容错码流，罕见 |
| Slice data partitioning（NAL 2–4） | 支持 | 不支持 | 罕见 |
| `separate_colour_plane_flag` | 支持 | 不支持 | 4:4:4 的一部分，罕见 |
| Redundant pictures | 支持/丢弃 | 丢弃（与参考一致） | 不影响正常输出 |
| 错误隐藏（EC） | 多种模式 | 未解码 MB 涂中灰 + `b_decode_error` | 损坏流画质不如 FFmpeg |
| 硬件加速（DXVA2/D3D11/NVDEC/QSV/VAAPI） | 支持 | 无 | 功耗/4K 实时场景差距大 |
| 容器解复用（MP4/MKV/TS） | 支持 | 示例只吃 Annex-B/AVCC 包 | 库本身不做 demux，属于集成层 |
| 输出像素格式 | 多种（NV12/P010/…） | 仅 8-bit YUV420P 平面 | 集成时需要转换 |

### 2.3 复现功能矩阵

新增了 `tests/feature_matrix.sh`：为每个 profile/chroma 组合编码 10 帧，分别用本解码器和 FFmpeg 解码，能出图就逐字节比较。

```sh
sh tests/feature_matrix.sh
```

本次输出（FFmpeg 9.0.2 + libx264）：

```
case                   ffprobe profile,pix_fmt  pix_fmt        result
baseline_420p8         Constrained Baseline,yuv420p yuv420p        PASS     bit-exact
main_420p8             Main,yuv420p             yuv420p        PASS     bit-exact
high_420p8             High,yuv420p             yuv420p        PASS     bit-exact
high10_420p10          High 10,yuv420p10le      yuv420p10le    PASS     bit-exact
high422_422p10         High 4:2:2,yuv422p10le   yuv422p10le    PASS     bit-exact
high444_444p10         High 4:4:4 Predictive,yuv444p10le yuv444p10le PASS bit-exact
high422_422p8          High 4:2:2,yuv422p       yuv422p        UNSUPPORTED
high444_444p8          High 4:4:4 Predictive,yuv444p yuv444p        UNSUPPORTED
```

10-bit 现在覆盖 4:0:0/4:2:0/4:2:2/4:4:4，`make check10` 会在 1/8/12 槽下与 FFmpeg 逐字节比较：19/19 流通过（High 10 CABAC/CAVLC、B 帧加权预测、MBAFF、PAFF、8 slices、transform bypass、720p，以及 4:2:2/4:4:4 的 CABAC/CAVLC、MBAFF、裁剪、无损和 monochrome 4:0:0）。

```sh
mingw32-make check10
```

### 2.4 High 10 实现要点

10-bit 不是简单地把输出变成 16 位；它改变了整条像素流水线的算术宽度：

- SPS 接受 `bit_depth_luma = bit_depth_chroma = 10`，帧平面改为 16-bit 样本，`linesize` 仍是字节，`bytes_per_sample` 导出为 2。
- 残差系数用 `int32_t` 存放（10-bit 反量化后可能超过 int16），CABAC/CAVLC 的块指针按 `16 * index << 1` 前进；8-bit 仍走原来的 int16 路径。
- MC、帧内、IDCT、加权预测、去块各有一套 `_16` 标量实现，裁剪到 0..1023；去块的 alpha/beta/tc 按 `1 << (bit_depth-8)` 放大，QP 表索引减去 `6*(bit_depth-8)`。
- CAVLC 的 `mb_qp_delta` 上界从 51 改为 `51 + 6*(bit_depth-8)`；PPS 的 `init_qp` 同样加上这个偏移。这两个值少一个都会让 10-bit 的 QP 每宏块漂移 12，残差整体小 4 倍（本实现第一次跑 10-bit 时正是这个症状）。
- Lossless/transform bypass 的 16-bit 残差加法按 `uint16_t` 回绕，和 FFmpeg 的 `ff_h264_add_pixels*` 一致；CAVLC I_PCM 的跳过位数从 `384*8` 改为 `384*bit_depth`。
- 整数位置 chroma 直接走 copy/avg（10-bit 下 90% 以上的 chroma MC 是整数位置），copy/avg 行内核对 x86-64 用 SSE2 `pavgw`。

验证：`make check10` 生成 8 条 High 10 流（CABAC/CAVLC、B 帧加权预测、MBAFF、PAFF、8 slices、transform bypass、720p），在 1/8/12 槽下全部与 FFmpeg 逐字节一致；10-bit 的 96 个 fuzz 变体无崩溃。

---

## 3. 性能对标

### 3.1 纯解码吞吐（丢弃图像）

**720p High，1500 帧**

| 线程/槽 | h264dec fps | FFmpeg fps | FFmpeg / h264dec | h264dec 加速比 | FFmpeg 加速比 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 653 | 784 | 1.20 | 1.00x | 1.00x |
| 2 | 1062 | 1362 | 1.28 | 1.63x | 1.74x |
| 4 | 1833 | 2419 | 1.32 | 2.81x | 3.09x |
| 6 | 2545 | 3290 | 1.29 | 3.90x | 4.20x |
| 8 | 3084 | 4000 | 1.30 | 4.72x | 5.10x |
| 10 | 3286 | 4573 | 1.39 | 5.03x | 5.83x |
| 12 | 3245 | 4951 | 1.53 | 4.97x | 6.31x |

**1080p High，1000 帧**

| 线程/槽 | h264dec fps | FFmpeg fps | FFmpeg / h264dec | h264dec 加速比 | FFmpeg 加速比 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 309 | 394 | 1.27 | 1.00x | 1.00x |
| 2 | 523 | 672 | 1.28 | 1.69x | 1.71x |
| 4 | 892 | 1188 | 1.33 | 2.89x | 3.02x |
| 6 | 1180 | 1587 | 1.35 | 3.82x | 4.03x |
| 8 | 1364 | 1835 | 1.35 | 4.41x | 4.66x |
| 10 | 1495 | 1988 | 1.33 | 4.84x | 5.05x |
| 12 | 1551 | 2088 | 1.35 | 5.02x | 5.30x |

**PAFF 720p，1000 个编码图像**

| 线程/槽 | h264dec fps | FFmpeg fps | FFmpeg / h264dec | h264dec 加速比 | FFmpeg 加速比 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 729 | 837 | 1.15 | 1.00x | 1.00x |
| 2 | 982 | 1499 | 1.53 | 1.35x | 1.79x |
| 4 | 1518 | 2558 | 1.69 | 2.08x | 3.06x |
| 6 | 1971 | 3226 | 1.64 | 2.70x | 3.86x |
| 8 | 2220 | 3509 | 1.58 | 3.05x | 4.19x |
| 10 | 2209 | 3623 | 1.64 | 3.03x | 4.33x |
| 12 | 2124 | 3663 | 1.72 | 2.91x | 4.38x |

要点：

1. **单线程差距最小**：720p 1.20x、1080p 1.27x、PAFF 1.15x。
2. **多线程差距被放大**：720p 12 线程 1.53x，PAFF 1.72x。
3. **PAFF 是最大短板**：h264dec 在 8 槽达到 3.05x 后不再上升，12 槽反而下降到 2.91x；FFmpeg 仍能到 4.38x。
4. 720p 在 10 槽达到峰值，12 槽略降；1080p 到 12 槽仍在缓慢上升。

### 3.2 High 10 纯解码吞吐

同一台机器，`long_720p_10.h264`（600 帧，High 10，CABAC），双方都丢弃图像：

| 线程 | h264dec fps | FFmpeg fps | FFmpeg / h264dec |
| --- | ---: | ---: | ---: |
| 1 | 390 | 591 | 1.52 |
| 4 | 1252 | 1835 | 1.47 |
| 8 | 2033 | 2469 | 1.21 |
| 12 | 2237 | 2679 | 1.20 |

10-bit 的单线程差距比 8-bit（1.20x）大，原因是 16-bit 像素路径目前大部分是标量实现：MC 的非整数位置、帧内、IDCT、去块都按 `uint16_t` 逐样本计算，没有完整的 SSE2/AVX2 版本；熵解码与 8-bit 共用同一套 CABAC/CAVLC。已经落地的一个 10-bit 优化是整数位置 chroma 的 copy/avg 快路径（与 8-bit 的 93% 整数位置现象一致），以及 16-bit copy/avg 的 SSE2 `pavgw` 行内核，带来单线程 +20%、8 线程 +15%。线程扩展仍然成立（1→12 槽 5.7x）。下一步的 10-bit 优化应按收益排序：先做去块和 IDCT 的 16-bit SSE2，再做 MC 的 int32 中间结果向量化。

### 3.3 文件 → YUV 的实际作业

**720p，两个程序都把 YUV 写入空设备**

| 线程 | h264dec fps | FFmpeg fps | h264dec / FFmpeg |
| --- | ---: | ---: | ---: |
| 1 | 633 | 755 | 0.84 |
| 2 | 1108 | 1254 | 0.88 |
| 4 | 1934 | 2149 | 0.90 |
| 6 | 2489 | 2389 | **1.04** |
| 8 | 2880 | 2143 | **1.34** |
| 10 | 2998 | 2193 | **1.37** |
| 12 | 2843 | 2206 | **1.29** |

**1080p，同样写 YUV**

| 线程 | h264dec fps | FFmpeg fps | h264dec / FFmpeg |
| --- | ---: | ---: | ---: |
| 1 | 293 | 372 | 0.79 |
| 2 | 523 | 591 | 0.89 |
| 4 | 859 | 883 | 0.97 |
| 6 | 1090 | 950 | **1.15** |
| 8 | 1154 | 917 | **1.26** |
| 10 | 1217 | 934 | **1.30** |
| 12 | 1193 | 932 | **1.28** |

原因不在解码器突然变快，而在输出路径：

- FFmpeg 写 rawvideo 时，8 线程从 4000 降到 2143 fps（**-46%**），12 线程从 4951 降到 2206（**-55%**）；
- h264dec 的示例写 YUV 时，8 线程从 3084 降到 2880（**-6.6%**），12 线程从 3245 降到 2843（**-12.4%**）。

本次还改了 `example_decoder.c` 的一处输出细节：给 YUV 文件设置 1 MB 的 `setvbuf` 缓冲区。720p 一帧要写 1440 行，默认 4 KB stdio 缓冲会让每几行变成一次 write 系统调用；1 MB 缓冲在 8/10 线程测得 +4%–5%，单线程约 +2%。这个改动只影响示例程序的输出路径，不影响解码库。

所以“作为解码器”和“作为文件转 YUV 的作业”是两个不同的排名。**后者在 8 线程以上已经领先 FFmpeg 25%–37%**（6 线程约持平），这也是 README 里 2%–26% 领先的同一现象在本次测量中的更新值。

### 3.4 CPU 与内存（1080p）

用 `Start-Process` 轮询采样，单位 MB / 秒：

| 线程 | h264dec 峰值 RSS | FFmpeg 峰值 RSS | h264dec 进程 CPU | FFmpeg 进程 CPU | h264dec 平均占用核数 | FFmpeg 平均占用核数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 47.6 | 71.2 | 3.25 | 3.92 | 0.97 | 1.54* |
| 8 | 91.7 | 100.4 | 5.06 | 4.58 | 6.88 | 8.40 |
| 12 | 113.3 | 122.9 | 6.13 | 5.06 | 9.17 | 10.57 |

\* FFmpeg 在 `-threads 1` 下仍有约 1.5 个线程的 CPU 时间，可能来自输入/输出或内部辅助线程，因此这一行不能按“单线程 CPU 效率”直接比较。

关键信号：

- **内存更省**：1080p 单线程 47.6 MB 对 71.2 MB（0.67x），12 线程 113.3 对 122.9（0.92x）。
- **多线程利用率更低**：8 线程时 h264dec 平均只跑 6.88 个核，FFmpeg 跑 8.40 个核；12 线程 9.17 对 10.57。差距主要不是“每个线程更慢”，而是**有核在等**。

---

## 4. 差距归因

### 4.1 单线程：熵解码是最大头

项目内已有的分阶段测量（60s 720p，单线程，按宏块统计）：

| 阶段 | 占比 |
| --- | ---: |
| 熵解码（CABAC/CAVLC） | 46.9% |
| 重建（MC、变换、帧内、cache） | 27.7% |
| 环路滤波（一行延迟） | 22.9% |
| slice 循环簿记 | 2.4% |

其中残差 token 解码占常规 bin 的 71.9%、bypass bin 的 89.3%。FFmpeg 的同一段是 C + x86 内联汇编，且它的 MC/IDCT/去块有更完整的汇编覆盖。

本次额外验证：

- 把 FFmpeg 的 x86 `get_cabac_inline_x86` 思路移植到本项目的 `H264CABAC` 布局，做成 bit-exact 的临时版本（含 RBSP 零填充和越界保护），在 720p 上**没有可测量收益**（640–652 fps，基线 641–668 fps，落在噪声内），因此没有合入。
- `-march=native` / `-march=x86-64-v2` / `-mtune=alderlake` / clang 18 构建，单线程都在噪声内（约 640–660 fps），没有可见提升。
- 这说明剩下的 20% 不是“编译器没开对”，而是**代码本身每 bin/每宏块的工作量**与 FFmpeg 的汇编实现有差距。

### 4.2 多线程：不是锁，是“等”和内存

README 已经证明：

- 行进度锁、filter 队列、trace 标志的修复把 12 线程从 3.5x 提升到约 5.1x；
- 12 个独立进程的总吞吐约 3400 fps，和 12 槽线程版 3343 fps 基本一致——**线程池本身几乎没有协调开销**；
- 继续加线程（14/20 槽）只多 1.5%，所以不是槽数问题。

那为什么 FFmpeg 能到 6.3x？

1. **参考行等待**：CABAC 使一帧内宏块严格串行，后续帧要等参考帧的行。FFmpeg 的帧线程调度能让更多帧同时处于“可解码”状态。
2. **提交顺序屏障**：本实现中一个 worker 完成像素工作后，必须等 `commit_next` 轮到它才能释放槽位、接下一份工作。晚完成的帧会占着 worker。README 的旧测量认为 commit 链很便宜，但那是在 filter 内联、等待更重的版本上；filter 拆出去后值得重新测。
3. **PAFF 场图像**：`dispatch_access_unit()` 对 `field_picture` 先 `h264_job_wait_all()`，然后在 owner 线程内联解码，整池会短暂排空。README 说把场图像改成普通 job 只测到 1.003x，但那是在旧调度器上；结合提交解耦值得再试。
4. **内存带宽**：12 个独立进程也只有 5.2x，说明这台机器上每帧的像素/元数据流量本身就是上限之一。FFmpeg 的每帧流量更低（它的峰值 RSS 更高，但共享参考帧的方式不同），所以能到 6.3x。

本次还在临时构建里用 `rdtsc` 拆了 owner 的 `send_packet`，720p、12 槽、1500 帧：

| owner 内阶段 | cycles | 占 owner 总时间 |
| --- | ---: | ---: |
| NAL 切分 / RBSP 反转义 / slice header 解析 | 0.165G | 10% |
| `h264_field_start`（含 `reset_meta`） | 0.650G | 40% |
| `h264_slice_init` | 0.016G | 1% |
| `dispatch_access_unit` 的实际工作 | 0.036G | 2% |
| 等空闲 job 槽（反压，不是串行工作） | 0.777G | 48% |
| 合计 | 1.634G | 100% |

也就是说 owner 真正串行的工作约 **0.58M cycles/帧**，而 12 槽的流水线节拍是 **1.09M cycles/帧**，owner 还有约 50% 余量；它近一半的“时间”是在等空闲 job 槽，这是 worker 已经占满的反压。**所以 12 槽的瓶颈不在 owner 解析，而在 worker 的像素/内存路径。** 这修正了“把 header 解析搬到 worker 就能大幅提升”的直觉——它可能值几个百分点，但不足以填平 1.5x。

### 4.3 PAFF：3.0x 天花板

`long_paff_720p` 有 1000 个编码图像，其中 415 个场图像。场图像：

- 每个场是半幅宏块行，但仍有完整的 slice header、参考列表、POC、DPB、输出排队；
- 两个场共享一个 `H264Frame`，第二个场的 header 会改写 `picture_structure`、`poc`、`have_*_field`；
- 当前实现选择“排空所有 job，owner 内联解码场图像”，简单且 bit-exact，但会周期性制造流水线气泡；
- 场编码的像素在内存中隔行分布，MC/去块的 cache 行利用率只有一半。

这三点叠加，使 PAFF 在 8 槽后不再扩展。FFmpeg 能到 4.38x，说明它的场图像调度/内存路径更好。

### 4.4 端到端为什么能赢

FFmpeg 的 `-f rawvideo` 输出路径在多线程下成为瓶颈：它把每帧 YUV 交给输出线程/协议栈，线程越多，输出端争用越严重。`h264dec_example` 的 `fwrite` 路径更简单，且解码和输出在同一调用栈上，缓存局部性更好。因此：

> **纯解码**：FFmpeg 赢；**文件→YUV**：h264dec 在 6 线程以上赢。

这不是“作弊”，而是集成方式不同。README 已经用“双方都丢弃”和“双方都写”两组数据把这件事说清楚，本次测量复现了同样的结论。

---

## 5. 如果要“超越 FFmpeg”，优先级应该怎么排

### P0：低成本、可立即做的

1. **PGO 构建**：项目已有 `make pgo`，README 记录单线程 +5.5%、10 线程 +3%。如果发布二进制允许带训练数据，这是最快的 5%。
2. **保持 SSE2/AVX2 现有内核**：AVX2 qpel 只值 0.6%，但已 bit-exact，保留即可；不要为了“用上 AVX2”重写已经内存受限的 copy/chroma 内核。
3. **输出路径继续优化**：当前端到端的优势来自这里。可以把 `h264dec_example` 的 `fwrite` 改成按行/大块写、避免 stdio 锁，进一步扩大 6–12 线程的领先。

### P1：多线程调度（目标 +10%–20%）

1. **提交与执行解耦**：worker 完成像素工作后先释放 job slot 去接下一份工作，提交由 owner 或任意空闲线程按 `seq` 顺序完成。需要把 `H264Job` 从 worker 结构里拆出来，或允许一个 worker 同时持有“已完成待提交”和“正在解码”两份状态。
2. **重新测量 PAFF**：在场图像上把 `h264_job_wait_all()` 换成“等待配对场”的精细同步，配合 P1.1 的提交解耦；如果仍然只有 1.003x，再放弃。
3. **槽数/亲和性**：14–16 槽在这台机器上可能只多 1.5%，但配合提交解耦可能不同；Windows 下可实验性地把 worker 绑到逻辑核，注意不要默认开启。

### P2：单线程（目标 +10%–20%，难度高）

1. **CABAC 语法层**：残差解码之外的模式/分区/参考/MVD 占 21%，每 bin 成本 62 cycles，比残差层的 27 cycles 高得多。优先看：
   - `fill_decode_caches` / `fill_decode_neighbors` 的每宏块调用（可尝试把热路径搬进 `h264_cabac.c` 或头文件内联）；
   - `h264_decode_mb_skip` 的调用链（81% 宏块是 skip，但函数在另一个编译单元）；
   - 上下文状态布局与 `sl->` 别名，减少每 bin 的 load/store。
2. **汇编**：如果 P2.1 不够，再考虑把 `get_cabac` 和 `decode_cabac_residual` 写成 x86-64 汇编。本次原型没有收益，说明不能照搬 FFmpeg 的宏；需要针对本项目的结构体布局和编译器输出重新写。
3. **像素阶段**：剩下最大的是去块的 bS/cache 填充（约 10.4%）和 MC（约 12.1%）。SSE2 已经覆盖主要路径，继续做 AVX2 的边际收益很小。

### P3：功能补齐（真正的“全面对标”）

按重要性排序：

1. **10-bit SIMD**：功能已经补齐，下一步把 16-bit 去块/IDCT/MC 做成 SSE2/AVX2，优先去块和 IDCT；当前 10-bit 单线程落后 FFmpeg 1.78x，主要就是标量 16-bit 像素路径。
2. **8-bit 4:2:2 / 4:4:4、12-bit**：10-bit 的 4:0:0/4:2:0/4:2:2/4:4:4 已经完成并逐字节验证；8-bit 非 4:2:0 和 12-bit 仍未接受。
3. **FMO / 数据分区**：老码流兼容性，优先级低。
4. **错误隐藏**：至少对丢失 slice 做参考帧拷贝/插值，而不是只涂灰。
5. **硬件加速**：如果目标是 4K/低功耗，DXVA2/D3D11VA 或 NVDEC 是另一条产品线，不是当前 C 解码器的自然延伸。

---

## 6. 建议的验收指标

如果后续要声称“超过 FFmpeg”，建议用下面几条硬指标，而不是单次 fps：

| 指标 | 当前 | 目标 |
| --- | ---: | ---: |
| 720p 单线程 / FFmpeg | 0.83x | ≥ 1.00x |
| 1080p 单线程 / FFmpeg | 0.79x | ≥ 1.00x |
| 720p 12 线程 / FFmpeg | 0.66x | ≥ 1.00x |
| PAFF 8 线程 / FFmpeg | 0.63x | ≥ 1.00x |
| 文件→YUV 8 线程 / FFmpeg | 1.26x | ≥ 1.25x（保持） |
| 1080p 峰值 RSS | 0.92x | ≤ 1.00x（保持） |
| 功能矩阵 | 4 PASS / 4 UNSUPPORTED | 8 PASS |

---

## 7. 复现清单

```sh
cd H:/mycode/h264dec/h264dec

# 正确性
mingw32-make -j8
mingw32-make check

# High 10 正确性（1/8/12 槽，8 条流）
mingw32-make check10

# 功能矩阵
sh tests/feature_matrix.sh

# 10-bit 纯解码吞吐
CLIP=../testdata/bench/long_720p_10.h264 FRAMES=600 \
  REPS=3 THREADS="1 4 8 12" sh tests/bench_vs_ffmpeg.sh

# 纯解码吞吐
REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh
CLIP=../testdata/bench/long_1080p.h264 FRAMES=1000 \
  REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh
CLIP=../testdata/bench/long_paff_720p.h264 FRAMES=1000 \
  REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh

# 文件 -> YUV
SINK=1 REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh
SINK=1 CLIP=../testdata/bench/long_1080p.h264 FRAMES=1000 \
  REPS=3 THREADS="1 2 4 6 8 10 12" sh tests/bench_vs_ffmpeg.sh
```

---

## 8. 风险与说明

- **许可证**：本项目算法和常量表派生自 FFmpeg（LGPL-2.1+），发布二进制需遵守 LGPL。本文的性能对比不改变这一点。
- **测试机负载**：Windows 桌面、杀毒、浏览器都会影响 fps；本文所有比值都来自同一时间窗口的交替运行，但不能替代目标硬件上的复测。
- **FFmpeg 版本**：9.0.2 是 MSYS2 构建，开启大量组件；其 H.264 解码器本身的汇编覆盖与发行版构建有关。换一个 `--disable-asm` 的 FFmpeg 会得到不同的单线程数字。
- **“全面超越”定义**：如果指“功能矩阵 + 单线程 + 多线程 + 端到端”全部领先，目前还差 P2 和 P3；如果指“实际文件转 YUV 作业”，在 6 线程以上已经领先。
