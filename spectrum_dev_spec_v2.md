# 项目开发方案 v2：Spectrum — ESP32-S3 多传感器 + 光谱 + 摄像头 Web 监测仪

> 交付给 Claude Code 实现。环境 **ESP-IDF v6.0**，芯片 **ESP32-S3**（建议 N16R8，需 PSRAM）。
> v2 变更：IMU 改走 **I2C**（腾出一路 UART）；新增 **H1 光谱仪（UART）**；新增 **USB UVC 摄像头**。
> 局域网内通过 **http://spectrum.local** 访问实时 Web 页面。

---

## 1. 目标与范围

单颗 ESP32-S3 同时接入 4 类外设并提供实时 Web 监测：

1. **IMU（6 轴，I2C）**：姿态角 + 加速度 + 角速度。
2. **TinyF 激光测距（UART）**：单点距离(mm) + 置信度。
3. **H1 光谱仪（UART）**：曝光状态/时间、47 项光度学参数、蓝光/近红外/植物光照参数、整条光谱曲线（可选 TM30）。
4. **UVC 摄像头（USB）**：MJPEG 视频流（受全速限制，低分辨率）。
5. WiFi STA + mDNS `spectrum.local` + HTTP/WebSocket，Web 页面实时展示以上数据并显示摄像头画面。

**非目标**（预留不实现）：数据录制/持久化、OTA、HTTPS/鉴权、光谱定标修改（0x23/0x27 流程）。

---

## 2. 接口分配与接线

ESP32-S3 资源：3 路硬件 UART、I2C、1 个 USB-OTG（全速）。最终分配：

| 接口 | 设备 | 引脚（建议，可改） | 备注 |
|---|---|---|---|
| **I2C** | IMU | SDA=GPIO8，SCL=GPIO9 | 100 kHz，主机轮询读寄存器 |
| **UART0** | 调试/日志 | TX=GPIO43，RX=GPIO44 | 摄像头占用 USB，故 console 必须走 UART |
| **UART1** | TinyF | RX=GPIO17，TX=GPIO18 | ASCII 行，115200 |
| **UART2** | H1 光谱仪 | RX=GPIO15，TX=GPIO16 | HEX 协议，115200 |
| **USB-OTG** | UVC 摄像头 | D−=GPIO19，D+=GPIO20（固定） | 全速 12Mbps，需对外供 5V VBUS |

电源/地（全部共地）：

- IMU：`5V` 脚接 **3V3**（使其 IO 为 3.3V，对 S3 安全）。
- TinyF：`5V`；**若其 TX 为 5V 电平，接 S3 RX 前做电平转换**。
- H1：按其手册供电（5V/3.3V 视模组而定），TX 电平同样需确认 3.3V 兼容，否则分压。
- 摄像头：USB 5V VBUS 由板上供电轨提供，注意电流。

引脚约束：避开 USB(19/20)、Strapping(0/3/45/46)、Flash/PSRAM(26–37，N16R8 八线 PSRAM 占 33–37)。所有引脚集中在 `Kconfig`/常量里配置。

> H1 的 `TRIG` 引脚仅在**触发模式**下需要硬件脉冲；本方案用**流模式 + 软件请求**，**不需要接 TRIG**。

---

## 3. 传感器协议

### 3.1 IMU — I2C（100 kHz，主机读寄存器）

模块上电约 **5 秒**自检；之后主机按"寄存器=功能码"读取，数据为小端：

| 读取 | 功能码寄存器 | 字节 | 解析 |
|---|---|---|---|
| 欧拉角 | `IMU_FUNC_EULER` | 12 | 3×float = roll,pitch,yaw（**弧度**，×57.29578 转度） |
| 加速度 | `IMU_FUNC_RAW_ACCEL` | 6 | 3×int16 ×(16/32767) → g |
| 角速度 | `IMU_FUNC_RAW_GYRO` | 6 | 3×int16 ×(2000/32767)×π/180 → rad/s |
| 四元数 | `IMU_FUNC_QUAT` | 16 | 4×float = w,x,y,z |

官方 I2C 例程为 `IMU_I2C_ReadEuler/ReadAccelerometer/ReadGyroscope/ReadQuaternion`，即"先写功能码寄存器、再读 N 字节"。

> **未确认常量**：IMU 的 **I2C 7 位从机地址** 及各功能码寄存器值，官方 PDF 未列、在源码包里。实现前确认；或上电后用 `i2c` 扫描总线确定地址（总线上只有 IMU 一个设备，无冲突）。

### 3.2 TinyF — UART（115200 8N1，ASCII 行）

量程约 20–4000 mm，置信度 0–62。每行一帧 `" <distance>, <confidence>\n"`：

| 字段 | 字节 | Hex 例 | 含义 |
|---|---|---|---|
| 帧头 | 1 | `20` | 空格 |
| distance | 1–5 | `33 32 37` | "327" mm |
| 分隔符 | 2 | `2C 20` | "`, `" |
| confidence | 1–2 | `36 31` | "61" |
| 帧尾 | 1 | `0A` | `\n` |

解析：按 `\n` 切行 → 找 `,` 分割 → 两段 `atoi()`。建议丢弃 `confidence < 5` 的样本。

### 3.3 H1 光谱仪 — UART（115200 8N1，HEX 协议）

**通用包格式**（所有数值 HEX，多字节"总包长/曝光时间/光谱值"等均为**低位在前**）：

- 命令包：`CC 01 | LEN(3,LE) | CMD(1) | DATA(n) | CHK(1) | 0D 0A`
- 返回包：`CC 81 | LEN(3,LE) | TYPE(1) | DATA(n) | CHK(1) | 0D 0A`，其中 `TYPE == 命令的 CMD`。
- `LEN` = **整包总字节数**（含包头、长度域、类型、数据、校验、结束符）。
- `CHK` = 校验位之前**所有字节求和取低 8 位**。
- 结束符固定 `0D 0A`。

**本方案需要的命令**（示例字节已含正确校验，可直接发）：

| 功能 | CMD | 完整命令字节 |
|---|---|---|
| 取光谱起止波长 | `0x0F` | `CC 01 09 00 00 0F E5 0D 0A` |
| 单帧光谱(无 TM30) | `0x32` | `CC 01 09 00 00 32 08 0D 0A` |
| 连续光谱(无 TM30) | `0x33` | `CC 01 09 00 00 33 09 0D 0A` |
| 单帧光谱(含 TM30) | `0x34` | `CC 01 09 00 00 34 0A 0D 0A` |
| 停止 | `0x04` | `CC 01 09 00 00 04 DA 0D 0A` |
| 设备信息(24B) | `0x08` | `CC 01 0A 00 00 08 18 F7 0D 0A` |
| 设曝光模式 | `0x0A` | `CC 01 0A 00 00 0A <mode> <chk> 0D 0A`（`00`手动/`01`自动） |
| 设曝光值(us) | `0x0C` | `CC 01 0D 00 00 0C <u32 LE> <chk> 0D 0A` |
| 取曝光值 | `0x0D` | `CC 01 09 00 00 0D E3 0D 0A` |
| 设工作模式 | `0x41` | `CC 01 0A 00 00 41 <mode> <chk> 0D 0A`（`00`流/`01`触发） |
| 睡眠进/出 | `0x40` | `CC 01 09 00 00 40 16 0D 0A`（无返回） |

> 设备开机默认**流模式**。本方案保持流模式、用**自动曝光（`0x0A`=`01`）**，按周期发 `0x32` 单帧请求。需要更稳的话改手动曝光（`0x0A`=`00` + `0x0C` 设曝光，触发模式约束 7–59ms 本方案用不到）。

**`0x0F` 返回**：`CC 81 0D 00 00 0F | WL_START(2,LE) WL_END(2,LE) | CHK 0D 0A`。例 340–1050：`54 01`(=340) `1A 04`(=1050)。

**`0x32`/`0x33` 返回帧（无 TM30）有效数据 8 段**（按序）：

1. 曝光状态 `uint8`：`00`正常 / `01`过曝 / `02`欠曝
2. 曝光时间 `uint32`(LE, us)
3. 光度学参数 47×`float`：`[X,Y,Z,x,y,u,v,u',v',CCT,Nit,r_ratio,g_ratio,b_ratio,DUV,Ra,R1..R15,Lp,HW,Ld,purity,SP,SDCM,k,lux,Ee,fc,CQS,GAI_EES,GAI_BB_8,GAI_BB_15,EML,M_EDI]`
4. 蓝光危害 1×`float`：`Eb`
5. 近红外 3×`float`：`[Red_Ee, Nir_EeA, Nir_EeB]`
6. 植物光照 16×`float`：`[PAR,Eca,Ecb,Eb,Ey,Er,Erb_Ratio,PPFD,PPFDb,PPFDy,PPFDr,PPFDfr,PPFDr_ratio,PPFDy_ratio,PPFDb_ratio,YPFD]`
7. 光谱系数 `int16` N：实际光谱值 = 接收值 / 10^N
8. 光谱数据 `uint16[]`(LE)：点数 = (该段字节数)/2

`0x34`/`0x35`（含 TM30）在第 6 段后、第 7 段前插入 **TM30 614×`float`**。

**帧大小与节流（重要）**：711 点时无 TM30 帧 = 1706 字节（`AA 06 00`），含 TM30 = 4162 字节（`42 10 00`）。115200 下分别约 **148ms / 361ms** 才传完，即无 TM30 最高 ~6fps、含 TM30 ~2.7fps。**因此光谱按 ~1Hz 单帧请求**，不要连续模式刷屏，给摄像头/WiFi 留带宽。

**波长轴**：由 `0x0F` 的起止波长与点数推出，`step = (WL_END-WL_START)/(N-1)`。340–1050/711 点 → 1nm 分辨率。

**接收解析**：UART2 的 RX ring buffer 设 ≥ 8192。同步到 `CC 81` → 读 3 字节 `LEN` → 再收 `LEN-5` 字节（含 TYPE+数据+CHK+`0D 0A`）→ 校验 `CHK`(校验前全部字节和低 8 位) 且尾部为 `0D 0A` → 按 TYPE 拆 8/9 段。

---

## 4. ESP-IDF v6.0 注意事项

1. **UART legacy 驱动仍可用**（`driver/uart.h`）；被移除的是 ADC/DAC/I2S/RMT/PCNT/MCPWM/Timer/Temp 等 legacy 驱动，与本项目无关。
2. **I2C**：用新版 `driver/i2c_master.h`（`i2c_new_master_bus` / `i2c_master_transmit_receive`），不要用已废弃的 legacy `i2c.h`。
3. **组件搬迁到组件仓库**：`cJSON`、`wifi_provisioning`(更名 `network_provisioning`)、`esp-mqtt` 不再内置。本项目 JSON 用 `snprintf` 手写，避免 cJSON 依赖。
4. **mDNS** 为托管组件：`idf.py add-dependency "espressif/mdns"`。
5. **USB UVC 摄像头**：用 esp-iot-solution 的 **`espressif/usb_stream`**（S3 全速 UVC+UAC 主机，仅 MJPEG），`idf.py add-dependency "espressif/usb_stream"`。
6. **告警即错误**（默认 `-Werror`）：bring-up 期可临时 `CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y`，交付前清零告警再关闭。
7. **WebSocket**：`CONFIG_HTTPD_WS_SUPPORT=y`。
8. **console**：摄像头占 USB，故 `Channel for console output` 选 **UART0**（不能用 USB-Serial-JTAG / USB CDC，二者与 USB-OTG 共用 PHY，不能与摄像头并存）。
9. **PSRAM 必开**：摄像头帧缓冲 + 光谱缓冲需要，`CONFIG_SPIRAM=y`（八线）。
10. libc 改 picolibc（透明）；最低 CMake 3.22.1。

---

## 5. 工程结构

```
spectrum/
├─ CMakeLists.txt
├─ sdkconfig.defaults
├─ partitions.csv
├─ main/
│  ├─ CMakeLists.txt            # 注册源 + EMBED_FILES web/index.html
│  ├─ idf_component.yml         # espressif/mdns, espressif/usb_stream
│  ├─ Kconfig.projbuild         # WiFi、引脚、光谱采样率、曝光等
│  ├─ app_main.c
│  ├─ state.h / state.c         # 共享数据 + 互斥（分高频/低频两块）
│  ├─ imu_i2c.c                 # IMU I2C 任务
│  ├─ tinyf_uart.c              # TinyF UART 任务
│  ├─ spectro_uart.c            # H1 光谱仪 UART 任务（命令构造/帧解析）
│  ├─ camera_usb.c              # usb_stream UVC 主机，MJPEG 帧回调
│  ├─ net.c                     # WiFi STA + mDNS(spectrum)
│  ├─ webserver.c               # /、/ws、/api/data、/api/spectrum、/stream(MJPEG)
│  └─ web/index.html
```

`main/idf_component.yml`：

```yaml
dependencies:
  idf: ">=6.0"
  espressif/mdns: "^1.8.0"
  espressif/usb_stream: "*"      # 版本以仓库可用为准
```

`main/CMakeLists.txt`：

```cmake
idf_component_register(
  SRCS "app_main.c" "state.c" "imu_i2c.c" "tinyf_uart.c" "spectro_uart.c"
       "camera_usb.c" "net.c" "webserver.c"
  INCLUDE_DIRS "."
  EMBED_FILES "web/index.html"
  REQUIRES driver esp_driver_i2c esp_wifi esp_netif esp_event nvs_flash
           esp_http_server mdns)
```

`sdkconfig.defaults`（起步）：

```
CONFIG_HTTPD_WS_SUPPORT=y
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
CONFIG_FREERTOS_HZ=1000
# bring-up 期临时：
# CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y
```

---

## 6. 固件架构

### 6.1 共享数据（分两块，避免大数据拖累高频通道）

```c
// state.h
typedef struct {                         // 高频块（IMU + TinyF）
  float roll,pitch,yaw, ax,ay,az, gx,gy,gz; bool imu_valid; int64_t imu_ts;
  uint32_t distance_mm, confidence;        bool tof_valid;  int64_t tof_ts;
} fast_state_t;

typedef struct {                         // 低频块（光谱，最多约 711 点）
  uint8_t  exposure_status; uint32_t exposure_us;
  float    cct,lux,ra,x,y,peak_nm,duv;     // UI 常用标量（其余 47 项可全存）
  float    photometric[47], nir[3], plant[16], eb;
  uint16_t wl_start, wl_end, n_points;
  uint16_t spectrum[1024];                 // 解析后已乘 10^-N（或存原始+N）
  bool     spec_valid; int64_t spec_ts;
} spec_state_t;
```

各用一个互斥/自旋锁保护；提供 `get_fast_snapshot()` / `get_spec_snapshot()`。超时无更新置 `*_valid=false`。

### 6.2 任务

| 任务 | 频率/优先级 | 职责 |
|---|---|---|
| `imu_task` | ~50–100Hz / 中 | I2C 读欧拉角+加速度+角速度，写 fast_state |
| `tinyf_task` | 跟随模块 / 中 | UART1 按行解析，写 fast_state |
| `spectro_task` | ~1Hz / 中 | UART2：启动时取波长范围/设曝光；循环发 `0x32`、收帧、解析、写 spec_state |
| `camera_task` | — / usb_stream 回调 | MJPEG 帧回调，缓存最新 JPEG（双缓冲，PSRAM） |
| HTTP server | 低 | `/`、`/ws`、`/api/*`、`/stream` |
| `ws_push_task` | motion 20Hz / spectrum 1Hz | 取快照、序列化、异步推送各客户端 |

`app_main`：`nvs_flash_init` → `state_init` → I2C/UART 初始化并建 imu/tinyf/spectro 任务 → `usb_stream` 初始化 camera_task → WiFi STA 连接 → mDNS 注册 `spectrum` → `webserver_start` → `ws_push_task`。IMU/光谱任务首次循环前 `vTaskDelay(5200ms)` 跳过 IMU 自检期。

### 6.3 spectro_task 流程

1. （可选）`0x41`=`00` 确保流模式；`0x0A`=`01` 自动曝光。
2. `0x0F` 取 `wl_start/wl_end`。
3. 循环（约 1Hz）：发 `0x32` → 收变长帧（用 LEN 域）→ 校验 → 拆 8 段 → 光谱值 ×10^-N → 由波长范围算各点波长 → 写 spec_state。监控 `exposure_status`（过/欠曝时可在 UI 标红）。

---

## 7. 网络：WiFi + mDNS

- WiFi STA 连入用户网络（与访问端同子网），SSID/密码来自 Kconfig，掉线自动重连。
- mDNS：`mdns_hostname_set("spectrum")` → `spectrum.local`；`mdns_service_add(NULL,"_http","_tcp",80,...)`。
- `.local` 在 macOS/iOS/Linux/Win10+ 原生可解析。访问 `http://spectrum.local/`。

---

## 8. 数据接口

### 8.1 WebSocket `/ws`（两类消息）

高频（~20Hz）：
```json
{ "type":"motion", "t":123,
  "imu":{"valid":true,"roll":1.2,"pitch":-3.4,"yaw":88.9,
         "ax":0.01,"ay":-0.02,"az":0.99,"gx":0,"gy":0,"gz":0},
  "tof":{"valid":true,"distance_mm":327,"confidence":61} }
```
低频（~1Hz，光谱新帧时）：
```json
{ "type":"spectrum","t":124,"valid":true,
  "exposure_status":0,"exposure_us":2500,
  "cct":5021,"lux":312.5,"ra":92.1,"x":0.345,"y":0.351,"peak_nm":451,"duv":0.001,
  "wl_start":340,"wl_end":1050,"n_points":711,
  "spectrum":[12,13,15, ...] }
```

### 8.2 REST（轮询/补充）

| 路径 | 说明 |
|---|---|
| `GET /` | 内嵌 `index.html` |
| `GET /api/data` | 最新 fast_state JSON（WS 不可用时回退） |
| `GET /api/spectrum` | 最新光谱完整 JSON（含 47 光度学/NIR/植物全字段） |
| `GET /stream` | 摄像头 `multipart/x-mixed-replace; boundary=frame` MJPEG 流 |

`/ws` handler 设 `.is_websocket=true`；推送用 `httpd_ws_send_frame_async` 遍历活动 fd（失败即剔除）。`/stream` 用 chunked 持续写 `--frame\r\nContent-Type: image/jpeg\r\n\r\n<jpeg>\r\n`。

---

## 9. Web UI（单文件、自包含、零外链）

**硬约束**：单个 `index.html`，内联全部 CSS/JS，**无任何外部资源**（设备所在网可能无外网）。仅用 Canvas / WebSocket / `<img>` MJPEG / 系统等宽字体栈。

布局（深色航空仪表 / 仪器风）：

1. 顶部状态条：WS 状态、设备名 `SPECTRUM`、运行时间。
2. **姿态**：人工地平仪(Canvas) + R/P/Y 数字。
3. **运动 + 测距**：加速度/角速度数字；大号距离、置信度条、量程条。
4. **摄像头**：`<img src="/stream">` 实时画面（注明全速低分辨率）。
5. **光谱**（整宽）：波长–强度曲线(Canvas，曲线下用可见光波长→RGB 渐变填充) + 读数 chips：CCT(K)、lux、Ra、x/y、峰值波长 Lp、曝光状态（过/欠曝标红）。

行为：加载即连 `ws://<host>/ws`，断线 1s 重连；WS 持续失败则改轮询 `/api/data`(1–2Hz) + `/api/spectrum`(0.5Hz)。`motion`/`spectrum` 两类消息分别更新对应区块；数据 >1s（光谱 >3s）无更新则降级 NO DATA。

参考实现（作为 `main/web/index.html` 起点）：

```html
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>SPECTRUM</title>
<style>
 :root{--bg:#0a0e14;--panel:#111722;--line:#1e2a3a;--ink:#cde3ff;--dim:#5d7088;
       --accent:#36e0c8;--warn:#ff5d5d;--good:#7CFFB2;
       --mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;}
 *{box-sizing:border-box}html,body{margin:0;height:100%}
 body{background:radial-gradient(1200px 600px at 50% -10%,#0f1726,#0a0e14 60%);
      color:var(--ink);font-family:var(--mono);letter-spacing:.02em}
 header{display:flex;justify-content:space-between;align-items:center;
        padding:10px 16px;border-bottom:1px solid var(--line);font-size:13px}
 .brand{font-weight:700;letter-spacing:.35em;color:var(--accent)}
 .dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--warn);
      margin-right:6px;vertical-align:middle}.dot.on{background:var(--good);box-shadow:0 0 10px var(--good)}
 main{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px;max-width:1040px;margin:0 auto}
 .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:14px}
 .card.wide{grid-column:1/-1}
 .card h2{margin:0 0 10px;font-size:11px;letter-spacing:.25em;color:var(--dim);text-transform:uppercase}
 canvas,img.feed{width:100%;display:block;border-radius:10px;background:#060a10}
 .row{display:flex;justify-content:space-between;font-size:13px;padding:4px 0}.row b{color:#fff}
 .big{font-size:42px;font-weight:700;color:#fff}.unit{font-size:15px;color:var(--dim);margin-left:6px}
 .bar{height:8px;border-radius:6px;background:#0c121c;border:1px solid var(--line);overflow:hidden;margin-top:6px}
 .bar>i{display:block;height:100%;background:var(--accent)}
 .chips{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
 .chip{background:#0c121c;border:1px solid var(--line);border-radius:8px;padding:6px 10px;font-size:12px}
 .chip b{color:#fff} .stale{opacity:.35} .nodata{color:var(--warn)}
 @media(max-width:720px){main{grid-template-columns:1fr}}
</style></head>
<body>
<header><span class="brand">SPECTRUM</span>
 <span><span id="dot" class="dot"></span><span id="stat">connecting…</span>
 &nbsp;·&nbsp;<span id="up">0s</span></span></header>
<main>
 <section class="card" id="attCard"><h2>Attitude</h2>
   <canvas id="adi" width="560" height="340"></canvas>
   <div class="row"><span>Roll / Pitch / Yaw</span><b id="rpy">—</b></div></section>
 <section class="card" id="motCard"><h2>Motion &amp; Range</h2>
   <div class="row"><span>Accel XYZ (g)</span><b id="acc">—</b></div>
   <div class="row"><span>Gyro XYZ (rad/s)</span><b id="gyr">—</b></div>
   <div style="margin-top:10px"><span class="big" id="dist">—</span><span class="unit" id="distU">mm</span></div>
   <div class="row"><span>Confidence</span><b id="cf">—</b></div>
   <div class="bar"><i id="cfBar" style="width:0%"></i></div></section>
 <section class="card wide"><h2>Camera (USB UVC · full-speed MJPEG)</h2>
   <img class="feed" id="cam" alt="camera" src="/stream"></section>
 <section class="card wide" id="specCard"><h2>Spectrum — H1</h2>
   <canvas id="spec" width="1000" height="280"></canvas>
   <div class="chips">
     <span class="chip">CCT <b id="cct">—</b>K</span>
     <span class="chip">lux <b id="lux">—</b></span>
     <span class="chip">Ra <b id="ra">—</b></span>
     <span class="chip">x,y <b id="xy">—</b></span>
     <span class="chip">Peak <b id="lp">—</b>nm</span>
     <span class="chip">Exp <b id="exp">—</b></span>
   </div></section>
</main>
<script>
const $=id=>document.getElementById(id), t0=Date.now();
setInterval(()=>$("up").textContent=Math.floor((Date.now()-t0)/1000)+"s",1000);

// artificial horizon
const c=$("adi"),g=c.getContext("2d");
function drawADI(roll,pitch){const W=c.width,H=c.height,cx=W/2,cy=H/2;
 g.clearRect(0,0,W,H);g.save();g.translate(cx,cy);g.rotate(-roll*Math.PI/180);
 const o=pitch*4;g.fillStyle="#2b6cff";g.fillRect(-W,-H+o,2*W,H);
 g.fillStyle="#5a3a1a";g.fillRect(-W,o,2*W,H);
 g.strokeStyle="rgba(255,255,255,.9)";g.lineWidth=2;g.beginPath();g.moveTo(-W,o);g.lineTo(W,o);g.stroke();
 g.restore();g.strokeStyle="#ffd23f";g.lineWidth=3;g.beginPath();
 g.moveTo(cx-55,cy);g.lineTo(cx-18,cy);g.moveTo(cx+18,cy);g.lineTo(cx+55,cy);g.stroke();}
drawADI(0,0);

// wavelength -> approximate RGB for spectral fill
function wlRGB(wl){let r=0,gg=0,b=0;
 if(wl>=380&&wl<440){r=-(wl-440)/60;b=1;}else if(wl<490){gg=(wl-440)/50;b=1;}
 else if(wl<510){gg=1;b=-(wl-510)/20;}else if(wl<580){r=(wl-510)/70;gg=1;}
 else if(wl<645){r=1;gg=-(wl-645)/65;}else if(wl<=780){r=1;}
 else{r=gg=b=.25;} if(wl<380||wl>780){r=gg=b=.3;}
 return `rgb(${r*255|0},${gg*255|0},${b*255|0})`;}
const sc=$("spec"),sg=sc.getContext("2d");
function drawSpec(d){const W=sc.width,H=sc.height,n=d.n_points,arr=d.spectrum;
 sg.clearRect(0,0,W,H); if(!n||!arr)return; let mx=1; for(const v of arr) if(v>mx)mx=v;
 const x=i=>i/(n-1)*W, y=v=>H-(v/mx)*(H-12)-4, wl=i=>d.wl_start+(d.wl_end-d.wl_start)*i/(n-1);
 for(let i=0;i<n-1;i++){sg.beginPath();sg.moveTo(x(i),H);sg.lineTo(x(i),y(arr[i]));
   sg.lineTo(x(i+1),y(arr[i+1]));sg.lineTo(x(i+1),H);sg.closePath();
   sg.fillStyle=wlRGB(wl(i));sg.globalAlpha=.55;sg.fill();sg.globalAlpha=1;}
 sg.strokeStyle="#cde3ff";sg.lineWidth=1.5;sg.beginPath();
 for(let i=0;i<n;i++){const px=x(i),py=y(arr[i]);i?sg.lineTo(px,py):sg.moveTo(px,py);}sg.stroke();}

const f=(v,n=2)=>(v>=0?"+":"")+(+v).toFixed(n);
let lastFast=0,lastSpec=0;
function onMotion(d){lastFast=Date.now();$("attCard").classList.remove("stale");
 if(d.imu&&d.imu.valid){drawADI(d.imu.roll,d.imu.pitch);
  $("rpy").textContent=[d.imu.roll,d.imu.pitch,d.imu.yaw].map(x=>f(x)).join(" / ")+"°";
  $("acc").textContent=[d.imu.ax,d.imu.ay,d.imu.az].map(x=>f(x)).join(" / ");
  $("gyr").textContent=[d.imu.gx,d.imu.gy,d.imu.gz].map(x=>f(x)).join(" / ");}
 if(d.tof&&d.tof.valid){const mm=d.tof.distance_mm;
  $("dist").textContent=mm>=1000?(mm/1000).toFixed(2):mm;$("distU").textContent=mm>=1000?"m":"mm";
  $("cf").textContent=d.tof.confidence+" / 62";
  $("cfBar").style.width=Math.min(100,d.tof.confidence/62*100)+"%";
  $("cfBar").style.background=d.tof.confidence<5?"var(--warn)":"var(--accent)";}}
function onSpectrum(d){lastSpec=Date.now();if(!d.valid)return;$("specCard").classList.remove("stale");
 drawSpec(d);$("cct").textContent=Math.round(d.cct);$("lux").textContent=d.lux.toFixed(1);
 $("ra").textContent=d.ra.toFixed(1);$("xy").textContent=d.x.toFixed(3)+","+d.y.toFixed(3);
 $("lp").textContent=Math.round(d.peak_nm);
 $("exp").textContent=({0:"OK",1:"OVER",2:"UNDER"})[d.exposure_status]+" "+(d.exposure_us/1000).toFixed(1)+"ms";
 $("exp").style.color=d.exposure_status?"var(--warn)":"#fff";}
setInterval(()=>{if(Date.now()-lastFast>1000)$("attCard").classList.add("stale");
 if(Date.now()-lastSpec>3000)$("specCard").classList.add("stale");},700);

let ws,poll;
const setStat=(on,t)=>{$("dot").classList.toggle("on",on);$("stat").textContent=t;};
function startPoll(){if(poll)return;poll=setInterval(async()=>{try{
  onMotion(await(await fetch("/api/data")).json());
  onSpectrum(await(await fetch("/api/spectrum")).json());setStat(true,"polling");}
  catch(e){setStat(false,"offline");}},1000);}
function stopPoll(){clearInterval(poll);poll=null;}
function connect(){try{ws=new WebSocket("ws://"+location.host+"/ws");}catch(e){startPoll();return;}
 ws.onopen=()=>{stopPoll();setStat(true,"live");};
 ws.onmessage=e=>{try{const m=JSON.parse(e.data);
   if(m.type==="spectrum")onSpectrum(m);else onMotion(m);}catch(_){}}; 
 ws.onclose=()=>{setStat(false,"reconnecting…");startPoll();setTimeout(connect,1000);};
 ws.onerror=()=>{try{ws.close();}catch(_){}}; }
connect();
</script></body></html>
```

---

## 10. 构建 / 烧录 / 访问

```bash
idf.py set-target esp32s3
idf.py add-dependency "espressif/mdns"
idf.py add-dependency "espressif/usb_stream"
idf.py menuconfig     # WiFi SSID/密码；console=UART0；SPIRAM=on；HTTPD_WS_SUPPORT=on
idf.py build
idf.py -p <PORT> flash monitor
```

访问 `http://spectrum.local/`。

里程碑（建议交付顺序）：

1. **M1 总线打通**：I2C 扫到 IMU 地址、UART1/UART2 收到原始字节（光谱仪发 `0x08` 能读回设备信息以验证链路与校验算法）。
2. **M2 解析**：IMU 出 R/P/Y；TinyF 出距离；光谱 `0x32` 单帧能完整收齐并按 8 段解析、光谱值正确（注意 10^N 与小端）。
3. **M3 摄像头**：usb_stream 枚举到 UVC 设备并拿到 MJPEG 帧（先串口打印帧率/大小）。
4. **M4 联网**：WiFi + mDNS，`ping spectrum.local` 通。
5. **M5 接口**：`/api/data`、`/api/spectrum`、`/stream` 正确。
6. **M6 WebUI + WS**：页面四区块实时刷新；断线重连/轮询回退正常；CPU/堆余量稳定。

---

## 11. 上线前必须确认的开放项

1. **IMU I2C 地址 + 功能码寄存器**（源码包或总线扫描确定）。
2. **TinyF / H1 的 TX 电平**：5V 则加电平转换再进 S3 RX。
3. **摄像头型号**：必须支持 **USB 全速 + MJPEG**，否则点不亮；分辨率/帧率以全速带宽为限（~VGA@15fps 量级）。需 5V VBUS。
4. **H1 曝光策略**：默认流模式 + 自动曝光；若现场光照动态大，确认是否改手动曝光并设值（`0x0C`）。
5. **WiFi 凭据**：menuconfig 配置，勿硬编码。
6. **欧拉角单位**：弧度 ×57.29578 转度；若读数已是 ±180 量级则去掉换算。
7. **引脚冲突复核**：避开 USB/Strapping/Flash-PSRAM。

---

## 12. 验收标准

- [ ] 上电 ~6s 后串口日志显示 IMU / TinyF / 光谱仪 / 摄像头四路均有有效数据。
- [ ] `http://spectrum.local/` 局域网可打开，无外网也正常渲染（无外链请求）。
- [ ] 姿态地平仪随翻转实时变化；测距随遮挡变化且置信度联动。
- [ ] 光谱曲线随光源变化，CCT/lux/Ra/x,y/峰值波长合理；过/欠曝时曝光状态标红。
- [ ] 摄像头画面在页面实时显示（接受全速低分辨率）。
- [ ] 光谱按 ~1Hz 刷新且不拖累 motion 的 ~20Hz 通道；整体 CPU/堆余量稳定无 OOM。
- [ ] WS 断开自动重连，重连期间轮询回退仍更新。
- [ ] `idf.py build` 在默认 `-Werror` 下零告警通过。
