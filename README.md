# xdma-driver-win

Xilinx XDMA 驱动（Windows x64）+ 零依赖 Python 封装，用于在另一台 Windows 机器上复现对 XDMA 板卡的寄存器访问与 DMA 搬运能力。

这套文件从 AC701（XC7A200T）的 Gen2 x4 工程里提取，也已确认可跨器件复用：换 FPGA 型号不影响本文档里的任何一项（详见「跨器件复用」）。

## 文件清单

| 文件 | 大小 | 作用 | 必需 |
|---|---|---|---|
| `XDMA.sys` | 27,376 B | 内核驱动本体（14.21.42.982） | 是 |
| `XDMA.inf` | 5,421 B | 安装信息，含 37 个设备 ID | 是 |
| `XDMA.cat` | 6,294 B | 签名目录文件，安装时校验用 | 是 |
| `XDMA.cer` | 788 B | 测试签名证书，**漏了它一定装不上** | 是 |
| `xdma.py` | 49,988 B | Python 封装（纯标准库，零依赖） | 是 |
| `install_driver.bat` | — | 提权安装脚本，含环境预检 | 否 |

MD5：`XDMA.sys` = `aa70a7df9289563748322638be75d8d5`

## 目标机要求

| 项目 | 要求 | 不满足的后果 |
|---|---|---|
| CPU 架构 | x64 | ARM64 / x86 无法加载 |
| 系统版本 | Windows 10 1507+ / Windows 11 x64 | 驱动要求 KMDF 1.15，Win7/8 不满足 |
| Python | 3.6+（`xdma.py` 用 f-string） | 导入报语法错 |
| 管理员权限 | 需要 | 装不了驱动 |
| **Secure Boot** | **必须关闭** | 测试签名驱动被拒，设备报 Code 52 |
| 测试签名模式 | `testsigning on` | 同上 |

## 安装

双击 `install_driver.bat`，UAC 点「是」。脚本会依次做五步：

```
[1/5] 读 UEFISecureBootEnabled，开着就直接退出并报 INSTALL_FAILED secureboot_on
[2/5] 把 XDMA.cer 导入 Root + TrustedPublisher 两个存储，回读确认
[3/5] bcdedit /set testsigning on
[4/5] pnputil /add-driver XDMA.inf /install
[5/5] 打印设备状态与已绑定驱动版本
```

如果第 3 步是新开的，**必须重启**。重启后：

```
python xdma.py list
```

能列出设备路径（末尾带 `#{74c7e4a9-...}`）就成功了。

## 为什么不能「拷过去双击就装」

这份 `.sys` 的签名链只有一层：

```
Issued to : WDKTestCert eoghanm,131680921125811315
Issued by : WDKTestCert eoghanm,131680921125811315    ← 自签根，不是公共 CA
Expires   : 2028-04-13
Timestamp : 无
```

它不是 Xilinx 的正式签名，也不是 WHQL / Attestation 签名。Windows 对内核驱动强制签名，**测试证书签名的驱动只在 testsigning 模式下才被接受**。所以证书和测试模式两个都得有，缺一不可：

| 证书导入 | testsigning | 结果 |
|---|---|---|
| 否 | 否 | 装不上（`pnputil` 拒绝或设备 Code 52） |
| 是 | 否 | 装不上（Code 52） |
| 是 | 是 | **正常** |
| 否 | 是 | 装上了但加载失败 |

## xdma.py 怎么用

驱动侧不提供专有 API，而是把每个功能节点注册成可按 GUID 枚举的设备接口。`xdma.py` 用 `ctypes` 调 Win32 把这层包成三个类，全是标准库。

```python
from xdma import XdmaDevice, Regs, DmaChannel, pattern_bytes

dev = XdmaDevice()                     # 按接口 GUID 找卡
print(dev.nodes())                     # 探测哪些节点存在

with Regs(dev, count=64) as r:         # 寄存器视图：索引 × 4 = 偏移
    print(hex(r.read(63)))             # 具体值取决于 FPGA 侧设计
    r.dump()                           # 一次系统调用读整片

with DmaChannel(dev, "h2c_0") as tx:
    tx.write(pattern_bytes(1 << 20), addr=0x80000000)
```

节点与 BAR 的对应关系由驱动按 IP 在配置空间里声明的映射分配，名字固定：

| 节点 | 对应 | 用途 |
|---|---|---|
| `control` | BAR0 | XDMA 自己的寄存器（通道、中断） |
| `user` | BAR2 | 用户 AXI4-Lite（需 IP 里开 `axilite_master_en`） |
| `h2c_0` | DMA Ch0 写 | Host → FPGA |
| `c2h_0` | DMA Ch0 读 | FPGA → Host |

几个已在实现里处理掉的坑，直接调用就不用重复踩：

- **单笔上限**：驱动单次 `ReadFile`/`WriteFile` 超过约 1~4MB 会挂死，内部自动切 1MB。
- **地址不自动推进**：XDMA 驱动每次操作后不更新指针，`DmaChannel` 内部每块都显式 `SetFilePointerEx`。
- **缓冲 4KB 对齐**：`aligned_buffer()` 返回 `(持有者, 视图)`，**持有者必须各自命名保留**，否则视图指向已回收内存，`memmove` 会访问违例。
- **填充缓冲只能用 `memmove`**：写成 `buf[:n] = data[...]` 会走逐元素 Python，H2C 掉到 17 MB/s，与数据量无关。
- **计时口径**：不要把 `DmaChannel.read/write` 的耗时当带宽，其中含分配与拷贝开销；测速要用裸 `ReadFile` + 预分配缓冲。

## 跨器件复用

换 FPGA 型号（例如从 A7-200T 到 K7-410T）时，本包**一行都不用改**，前提是保持这几个参数：

| 条件 | 说明 |
|---|---|
| `pf0_device_id` 保持 `7024` | 改了 INF 就匹配不上，驱动不绑定，`xdma.py` 报「未找到 XDMA 设备」。这是唯一会让整个包失效的操作 |
| `axilite_master_en = true` | 不开就没有 `user` 节点 |
| `C_PCIEBAR2AXIBAR_n = 0` | 保持 Host 偏移 ≡ AXI 地址，`Regs` 的索引→偏移换算才成立 |
| 至少一对 DMA 通道 | 缺了 `DmaChannel` 打开失败 |

`xdma.py` 不硬编码 VID/DID、不硬编码实例路径，只按接口 GUID 枚举，所以实例号变化、BAR 编号变化都无感。工程侧（`.xci` 的 `C_FAMILY`、XDC 引脚与时钟约束）则需要按新器件重做，那属于 FPGA 工程，与本包无关。

注意 7 系列集成的 PCIe 硬核只到 **Gen2**，没有 Gen3。

## 已知限制

**1. 证书 2028-04-13 到期，且签名没有时间戳。** 到期之后这份 `.sys` 可能无法再加载或重新安装，届时需要用新证书重签，或者换用正式签名的驱动。

**2. `%12%\XDMA.sys` 是共享路径。** 所有 `oemXX.inf` 都指向同一个 `C:\Windows\System32\drivers\XDMA.sys`。如果目标机已经装过别的 XDMA 驱动包，装这个会覆盖它 —— 判断实际生效的是哪份二进制，看文件大小即可。

**3. 不要改 `xdma.py` 里的 `XDMA_GUID`。** 它必须和 `.sys` 里编进去的接口 GUID 一致（`{74c7e4a9-6d5d-4a70-bc0d-20691dff9e9d}`）。改 INF 不影响这个值，只有重编 `.sys` 才可能变。注意 INF 里那个 `{a3a4c1ce-...}` 是安装类 ClassGuid，与接口 GUID 无关。

**4. `user` 节点的可用范围要看 BD。** BAR2 的 aperture 可能远大于实际分配的 AXI 段（例如 aperture 64KB 但只划了 4KB 给从设备），越界访问会落到互连默认从设备回 DECERR，读写都可能报错。

## 许可与来源

`XDMA.sys` / `XDMA.inf` / `XDMA.cat` 为 Xilinx 版权，取自 Xilinx 官方 Windows 驱动发行包（v14.21.42.982），此处仅作二进制再分发，未做修改。`xdma.py` 与 `install_driver.bat` 为本工程自写。

如需长期依赖，建议从 AMD/Xilinx 官方渠道获取驱动并按其许可条款使用。
