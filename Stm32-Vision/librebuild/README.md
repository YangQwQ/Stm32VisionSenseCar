# librebuild/ — 自编译 esp32 内核库（可复现）

大脑板（`Stm32-Vision`）的 AI 客户端直连 TLS，官方预编译内核库**不满足**本工程的三类需求，
必须自编 `esp32s3-libs` 再覆盖到 Arduino15。本目录把这套流程所需的**脚本 + 定制配置 + 链接脚本补丁**
收进版本库，使换机器 / 重装 IDE / 升级内核后能重建出同一份库。

> 原先这些散在 `.trae/`（已 `.gitignore`，不入库）。`.trae/README.md` 是更长的排查过程记录，
> 本文件是可执行的**结论**。

## 一、要改什么（定制配置）

四处定制。**库侧**的三处已落到下面的 `configs/`（不再只是某次生成物里手改，重建不会丢）；
**第四处是生效头文件里的缓冲区个数**，它不走库、overlay 也不覆盖，见本节末的
「但"缓冲区个数"不在 defconfig 里」。

| 配置 | 作用 | 不做会怎样 |
|---|---|---|
| `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`（并关 `CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC`） | TLS 记录缓冲与握手结构搬去 PSRAM（8MB 充足），内部堆只需少量连续块即可握手 | 内部堆碎片化（`freeHeap`/`maxBlock` 低）时 AI 请求频繁 `HTTPCLIENT_ERROR_SEND_PAYLOAD_FAILED(-3)` |
| `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN=8192`（+ `ASYMMETRIC_CONTENT_LEN=y`） | 减小握手所需的内部 RAM **连续块** | 握手失败 `-32512` / `-17040` |
| `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` + `CACHE_TX_BUFFER_NUM=32`（**PSRAM**，静态 TX 缓冲**保持 8 不动**） | UDP 图传突发不再被 8 个静态 TX 缓冲卡死（一帧 20~29KB ≈ 15~21 个 1400B 包）；cache 队列是驱动 TX 池满时的溢出吸收路径 | `enomem ≈ 2~2.4× sendOk`，帧发不完、帧率忽高忽低、速率被拖到 MCS0 |
| `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=32768`、`TCP_WND_DEFAULT=16384` | 流模式下 40KB 级请求体 `write()` 才不超时 | 大请求体发送超时 |

> ⚠️ **TX 突发该调 `CACHE` 而不是加大 `STATIC_TX`**：静态池是"上层来一帧就拷一份进去"，池满
> 就没了；cache 队列是驱动拿不到静态缓冲时的**排队待发**路径，正是为吸收突发设计的。
> 实测：加 `CACHE=32` 后 `enomem/sendOk` 4.87 → 0.12、帧率 5.4 → 24~26fps。
> （另：`STATIC_TX` 每个约 1.6KB 只是 Kconfig 的说法——*"Each buffer takes approximately 1.6KB
> of RAM"*，**没写"内部"**；本配置下缓冲池走 PSRAM 优先的分配器，见下方 ⚠️。别再引用
> "8 个占内部 12.8KB"这个说法，那是我早先的误推。）

`configs/defconfig.esp32s3` 里对 WiFi 那几项写了**逐条源码级注释**（IDF v5.5.5 位置已核实：
`wifi_netif.c` 的 `wifi_transmit_wrap()`、`lwipopts.h` 的 `mem_clib_malloc`、`wifi_init.c` 的
`esp_wifi_psram_check()`），改之前先读那里，别只看本表。

> ⚠️ `esp_wifi_psram_check()`：开了 SPIRAM 就要求 `cache_tx_buf_num != 0`，否则 `esp_wifi_init` **直接失败**。
> 所以 `CACHE_TX_BUFFER_NUM=32` 与 `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` 是绑定的，不能只留一个。
>
> 注：`defconfig.esp32s3` 的 diff 里还有 `ENABLE_CHIPOBLE=n`、`MATTER_MAX_DYNAMIC_ENDPOINT_COUNT=4`
> 两项，那是上游为省 RAM 做的 Matter 取舍，与本工程无关，跟着上游即可。

### configs/ 里三份文件相对上游都是**修改态**

`esp32-arduino-lib-builder` 是 git clone；下面三份是在它的**工作区**里改的，
`git status` 能看到 `M`。**重新 clone 就会丢**，所以必须入库（本目录就是那份备份）：

| 文件 | 状态 |
|---|---|
| `configs/defconfig.common` | 改（mbedTLS 三项 + lwIP TCP 缓冲） |
| `configs/defconfig.esp32s3` | 改（WiFi 缓冲池 + 顺带的 Matter 取舍） |
| `configs/builds.json` | 改（`mem_variants_files` 段） |

`run-idflibs.sh` 里 `CFG=` 列的另外三份（`defconfig.debug_default`、`defconfig.qio`、`defconfig.80m`、
`defconfig.qio_ram`）是**上游原样**，不需要备份。

### ⚠️ 但"缓冲区个数"不在 defconfig 里——它在生效的头文件里

上表里 WiFi 那两项**必须分成两半看**，否则会白编几小时库还以为改好了：

| | 决定什么 | 改哪里 |
|---|---|---|
| `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` | 库里的**代码路径**存不存在（零拷贝 TX / PSRAM lwIP / cache 队列） | 库 ✅ defconfig + 重编 |
| `*_BUFFER_NUM` 那**几个数** | 运行时传给 `esp_wifi_init` 的字段值 | **头文件** ❌ 重编库无效 |

原理：`esp_wifi.h` 的 `WIFI_INIT_CONFIG_DEFAULT()` 把 `CONFIG_ESP_WIFI_*` 展开进
`wifi_init_config_t`，而这个宏是**在调用方（草图，经 `WiFiGeneric.cpp`）编译时**展开的。
所以个数是**头文件驱动**的；库自己那份 sdkconfig 里的 `STATIC_RX=?` 对运行时**无效**。

生效的那份是 **`dio_opi/include/sdkconfig.h`**（本板 `PSRAM=opi` + `FlashMode=dio` ⇒ 变体 `dio_opi`；
顶层 `include/` 下压根没有 `sdkconfig.h`）。而 `overlay-libs.sh` 只拷 `lib/ include/ flags/ ld/`
和 `sdkconfig` —— **从不碰变体目录**，所以这份改动和 `sections.ld` 一样是**手工打、重建后必须重打**。
已入库为 `patches/sdkconfig.h.dio_opi`。

当前值（本工程相对上游的改动）：

| 项 | 值 | 为什么 |
|---|---|---|
| `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM` | **16**（上游 8） | 静态 RX 缓冲在 `esp_wifi_init()` 时**一次性**分配（那会儿堆干净、无碎片），此后不释放；WiFi 硬件用它收**所有** 802.11 帧。静态不够才回落到 dynamic 池，而 dynamic **按需分配**，正撞运行期碎片 ⇒ 分不到就丢包 ⇒ 不回 ACK ⇒ ARP/ping/TCP 全哑且**不自愈**。所以哪怕下行只有 WS 小包也要留足静态。16 即 IDF 在本配置下的默认值。 |

> ⚠️ **"个数"不吃内部 RAM**（我曾写成"占内部 DMA RAM"，是错的）：开了
> `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` 后，WiFi 驱动的缓冲池走
> `esp_wifi/esp32s3/esp_adapter.c` 的 `wifi_malloc`/`wifi_calloc`/`wifi_realloc`
> —— 注释原文 *"Prefer to allocate a chunk of memory in SPIRAM firstly. If failed, try to
> allocate it in internal memory then."*（`heap_caps_malloc_prefer(size, 2, SPIRAM, INTERNAL)`）。
> **实测证据**：+8 个静态 RX 缓冲后，内部空闲堆 `29k` **一字节未变**。
> 即驱动按用途分两套分配器（`osi_funcs._malloc = malloc` 走内部、`._wifi_malloc` 走 PSRAM 优先），
> 缓冲池属后者。**所以"内部 RAM 水位"与"能否分到 RX 缓冲"不是同一条因果链**——
> 内部堆压力确实与小车的 WiFi 卡死相关（把 16KB 任务栈搬到 PSRAM 后谷底 1k→10k、卡死消失），
> 但**不是**通过"RX 缓冲分不到"这条路径，具体机理仍未查清，别再照那个说法推。
| `CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM` | 8（**不动**） | 见上：内部 RAM 常驻，TX 突发该走 CACHE |
| `CONFIG_ESP_WIFI_CACHE_TX_BUFFER_NUM` | 32 | 图传 TX 溢出吸收，落 PSRAM |

> ⚠️ `cache_tx_buf_num > 0` 决定 `WIFI_ENABLE_CACHE_TX_BUFFER` 位，而 `esp_wifi_psram_check()`
> 见到该位却 cache 数为 0 会直接 `ESP_ERR_NOT_SUPPORTED` 让 WiFi 起不来 ——
> **库与头文件必须一致地改**，别只改一边。

**验证改动真的进了固件**（"编译通过"不算证据，静默无效才是常态）：

```bash
uv run python tools/probe_macro.py            # 打印固件编译期的真实宏展开值
```

它从草图缓存的 `compile_commands.json` 取一条真实编译命令的完整参数，换成 `-E -dM` 只跑预处理，
打出来的 `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM` 就是**这次编译真正会用的值**。

## 二、怎么重建（四步）

> **路径约定**（本文档不写死任何用户名/盘符，换机器只需改这两行）：
>
> ```bash
> export R=<本仓库路径>      # WSL 里形如 /mnt/d/Git/VisionSenseCarProject
> export A15=<Arduino15 路径> # Windows 的 %LOCALAPPDATA%\Arduino15，WSL 里形如
>                            #   /mnt/c/Users/<你的用户名>/AppData/Local/Arduino15
> ```
>
> ⚠️ 这些命令请**在同一个 WSL shell 会话里连续执行**：每条都从 Git Bash 起一次
> `wsl.exe bash -lc '...'` 会丢变量（见「三、坑」最后一条）。
>
> **两个脚本都不写死路径**（没有 `/home/<某人>`、`C:\Users\<某人>` 这种东西）：
> 未设环境变量时，它们会在**开头交互提问**，回车即采用中括号里的默认值；默认值本身
> 都与用户名无关（`$HOME/...`，或从 Windows 的 `%LOCALAPPDATA%` 自动推）。
> 想免提问就导出变量（`IDF_PATH` / `LIB_BUILDER` / `ESP32S3_LIBS_SRC` / `ESP32S3_LIBS_DST` /
> `ESP32_CORE_VER` / `IDFLIBS_LOG`）——**非交互运行（管道 / CI）会自动跳过提问、直接用默认值**。
> `overlay-libs.sh` 覆盖前还会要一次 `Y/n` 确认（它会盖掉 IDE 已装的库）。

### 第 1 步：WSL 里编库

前置：`esp-idf` 在 `~/esp-idf`，lib-builder 在 `~/esp32-arduino-lib-builder`
（不同位置就 `export IDF_PATH=…` / `export LIB_BUILDER=…`）。
先把本目录 `configs/` 覆盖回 lib-builder（**重 clone 后必须做这步**）：

```bash
cp -f "$R/Stm32-Vision/librebuild/configs/"* ~/esp32-arduino-lib-builder/configs/
```

再跑 `run-idflibs.sh`（等价于 `build.sh` 的 esp32s3 全量配置，但绕过其中必失败的
`srmodels`/`bootloader`/`mem-variant` 几步）：

```bash
bash "$R/Stm32-Vision/librebuild/run-idflibs.sh"
```

成功判据（脚本自己会校验，退出码 0）：产物 `out/tools/esp32-arduino-libs/esp32s3/` 里
`lib/libmbedtls.a` 存在，且该目录 `sdkconfig` 含 `SSL_IN_CONTENT_LEN=8192`。

> ⚠️ 用 `build.sh` 时**绝对不要加 `-b build`**——`-b build` 是"只编 demo app 用于上传"，
> **不会刷新** `out/.../lib/` 里的静态库（`libmbedtls.a` 时间戳留在旧值），极易误判成功。
> 不带 `-b` 才真正把新配置写进 `out/`。改配置后重跑要先删产物（`rm -rf build sdkconfig out`）。
> 本目录的 `run-idflibs.sh` 每轮已自带 `rm -rf build sdkconfig`。

### 第 2 步：覆盖到 Arduino15（Windows 侧）

`overlay-libs.sh` 把产物覆盖进已安装的库包：

- 源：`~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32s3`
- 目标：`$A15/packages/esp32/tools/esp32s3-libs/<内核版本>`
  （当前 `3.3.11`；**目录名必须与已装 esp32 内核版本一致**，升级内核后要同步改）

它覆盖 `lib/ include/ flags/ ld/` 四个目录 + `sdkconfig`，并清理套嵌目录、打印归档数供核对。

### 第 3 步（易漏）：生效的头文件 `dio_opi/include/sdkconfig.h`

**`overlay-libs.sh` 不管这个**（它只覆盖 `lib/ include/ flags/ ld/` + `sdkconfig`）。重装内核 /
重跑 overlay 之后，把入库的补丁拷回生效变体目录：

```bash
cp -f "$R/Stm32-Vision/librebuild/patches/sdkconfig.h.dio_opi" \
      "$A15/packages/esp32/tools/esp32s3-libs/3.3.11/dio_opi/include/sdkconfig.h"
```

再用 `tools/probe_macro.py` 核一遍（见上）。**改完头文件必须重编草图**，且建议 `--clean`
（它躺在 74 个文件的 `.d` 依赖里，增量构建不保证认这个变更）：

```bash
uv run tools/carctl.py build --clean --flash
```

### 第 4 步（易漏）：链接脚本补丁 `sections.ld`

⚠️ **`overlay-libs.sh` 不管这个**——它只覆盖 `lib/ include/ flags/ ld/` 和 `sdkconfig`，
而链接脚本在**变体目录**里（`dio_opi/`、`qio_opi/` 等；本板用 `PSRAM=opi` + `FlashMode=dio`
⇒ 生效的是 **`dio_opi/sections.ld`**）。所以这份补丁是**手工打的，重建/升级后必须重打**。

已入库的是当前生效版本：`patches/sections.ld.dio_opi`。它与上游的差异只有给 RMT 驱动的
`__func__` 字符串加了去重段：

```
*libesp_driver_rmt.a:rmt_tx.*(.rodata.__func__*)
*libesp_driver_rmt.a:rmt_tx.*(.rodata.__FUNCTION__*)
```

重打方式：升级内核后，取新的 `dio_opi/sections.ld`，把上面两行加进去，覆盖到
`.../esp32s3-libs/<版本>/dio_opi/sections.ld`。（其余四个变体目录本板用不到，未打。）

## 三、坑（都踩过）

- **别读错 sdkconfig**：`esp32s3-libs/3.3.11/sdkconfig`（Windows 安装目录里的那份）**不是**编库时
  用的配置快照，别拿它判断某个 CONFIG 开没开。真配置在 WSL 产物里：
  `~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32s3/sdkconfig`。
  曾据此误判 `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` 没开、差点白跑几小时重编——**它早就是 `=y`**。
- **驱动日志在编译期就被摘掉了**：`CONFIG_LOG_MAXIMUM_LEVEL=1` ⇒ WiFi/lwIP 的 `ESP_LOGW/I/D`
  根本不在库里，**运行时 `esp_log_level_set` 救不回来**。想看见驱动内部告警必须重编库
  （把 `LOG_MAXIMUM_LEVEL` 抬到 3+）——这是目前唯一还值得重编的理由。
- **Git Bash 调 WSL 会丢内联变量**：从 Git Bash 起 `wsl.exe bash -lc 'T=<你的仓库路径>; cp x $T/'`
  时，`$T` 会变空，报 `目标 '/configs/' 不存在`（命令串被 Git Bash 先吃了一遍）。
  **两种安全写法**：① 先 `wsl` 进交互会话，在里面 `export`（变量活在 WSL 侧，本文档「二」用的就是这种）；
  ② 或用字面绝对路径。**只有"每条命令都从 Git Bash 起一次 `wsl -lc`"才不能用变量**。
- **升级内核版本后**：`ESP32_CORE_VER`（`overlay-libs.sh` 的目标版本，默认 `3.3.11`）、
  `patches/` 的两份变体补丁、以及 `Stm32-Vision/CLAUDE.md` §构建要点 的 fqbn 都要同步核对。

## 目录

| 路径 | 说明 |
|---|---|
| `run-idflibs.sh` | 【WSL】编库入口（重试 3 次 + 校验 8192）；路径未配置则开头提问 |
| `overlay-libs.sh` | 【WSL】产物覆盖到 Arduino15（含套嵌清理 + 统计 + `Y/n` 确认）；路径未配置则开头提问 |
| `configs/defconfig.common` | 定制：mbedTLS + lwIP TCP 缓冲 |
| `configs/defconfig.esp32s3` | 定制：WiFi TX 缓冲池（带源码级注释） |
| `configs/builds.json` | 定制：`mem_variants_files` |
| `patches/sdkconfig.h.dio_opi` | 当前生效的变体头文件（`STATIC_RX=16`；overlay 不覆盖它） |
| `patches/sections.ld.dio_opi` | 当前生效的链接脚本（含 RMT `__func__` 去重补丁） |

相关工具（在 `Stm32-Vision/tools/`）：
`probe_macro.py`（验宏是否真进固件）、`carctl.py`（`build --flash` 一条龙编译+OTA+核指纹）。
