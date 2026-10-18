# -*- coding: utf-8 -*-
"""
xdma.py — Xilinx XDMA 驱动的 Python 封装（零第三方依赖，仅用 ctypes 调 Win32 API）

======================================================================
它解决什么
======================================================================
XDMA 官方只提供 C 版命令行工具（xdma_rw.exe / streaming_dma.exe），
要在 Python 里做数据搬运、控制 FPGA、测带宽，只能自己调驱动。
本模块把这件事封装成几个类，让上层程序像用普通对象一样用 XDMA：

    from xdma import XdmaDevice, Regs, DmaChannel, loopback

    dev = XdmaDevice()                      # 打开第一块 XDMA 卡
    print(dev.nodes())                      # 驱动暴露了哪些节点

    with Regs(dev, count=64) as regs:       # BAR 寄存器区
        regs.write(0, 0xDEADBEEF)
        print(hex(regs.read(0)))

    with DmaChannel(dev, "h2c_0") as h2c, DmaChannel(dev, "c2h_0") as c2h:
        h2c.write(b"hello fpga")

======================================================================
驱动模型（务必理解，否则会写出死锁代码）
======================================================================
驱动把每块卡暴露成一组「命名管道式」的文件节点，路径形如（Windows 设备路径）：

    <设备路径>\\<节点名>       例如 ven_10ee&dev_7024 那条 PCI 路径 + \\user

实际拿到手的长这样（开头的两个反斜杠问号是 Win32 设备路径前缀）：

    ?\\pci#ven_10ee&dev_7024#3&2f0e5a00&0&00#{74c7e4a9-6d5d-4a70-bc0d-20691dff9e9d}\\h2c_0

节点语义：

    control    BAR0  ——  XDMA 自己的配置寄存器（通道开关、IRQ 等）
    user       BAR2  ——  用户 AXI4-Lite 寄存器区（本工程与 XDMA_LB 恒等映射）
    h2c_N      ——     Host to Device 描述符 DMA（AXI-Stream 模式下即 M_AXIS_H2C）
    c2h_N      ——     Device to Host 描述符 DMA（即 S_AXIS_C2H）
    event_N    ——     MSI 事件

访问方式全部是标准 Win32：CreateFile 打开 → SetFilePointerEx 定位 → ReadFile/WriteFile。

======================================================================
⚠️ 三条铁律（都是踩过坑的）
======================================================================
1) 【纯 AXI-Stream 回环必须并发】
   FPGA 侧若把 h2c_0 的输出直接接回 c2h_0 的输入（无 DDR 缓冲），
   那么 H2C 写必须与 C2H 读**同时进行**：C2H 的 ReadFile 一发出才会拉高
   tready，H2C 才有去处。顺序执行必然死锁。
   → 本模块的 loopback() / bandwidth() 已内建双线程并发，直接用即可。

2) 【BAR 越界 = 蓝屏】
   驱动用 MmMapIoSpace 把 BAR 映射成非分页虚拟地址，进程再去读映射长度
   之外的地址 ⇒ 非分页区页错误 ⇒ Windows 0x50 蓝屏。
   → Regs 类默认强制 window=0x1000，越界直接抛异常，绝不试探。
   要放开必须先从 xci 静态读出真实映射长度。

3) 【单笔传输有上限】
   实测单次 ReadFile/WriteFile 超过约 1~4 MB 会挂死（驱动内部上限）。
   → 本模块内部一律按 CHUNK=1MB 自动分块，对上层透明。

======================================================================
命令行（等价于 import 使用，方便快速验证）
======================================================================
    python xdma.py list                 # 枚举设备与节点
    python xdma.py info                 # 各节点是否存在 + BAR 探测（安全范围内）
    python xdma.py regs --dump          # 读回 user 节点前 64 个寄存器
    python xdma.py regs 0               # 读 0 号寄存器
    python xdma.py regs 0 0x1234        # 写 0 号寄存器并读回校验
    python xdma.py lb --size 1048576    # 回环 1MB 并校验数据
    python xdma.py bw --size 268435456  # 256MB 带宽测量（双向并发）

作者：AC701 + XDMA 工程验证沉淀（2026-10-16）
"""
from __future__ import absolute_import, print_function

import argparse
import ctypes
import struct as _struct
import sys
import threading
import time
from ctypes import wintypes

__all__ = [
    "XdmaError", "XdmaNotFound", "BarAccessError",
    "XdmaDevice", "Regs", "DmaChannel",
    "loopback", "bandwidth", "LoopbackResult", "BandwidthResult",
    "pattern_bytes", "is_admin",
]

__version__ = "1.2"

# ==================================================================== 常量
CHUNK = 1 << 20            # 内部 DMA 分块 1MB（规避驱动单笔上限）
ALIGN = 4096               # XDMA 对 DMA 缓冲的硬性对齐要求
DEFAULT_WINDOW = 0x1000    # BAR 安全访问窗口上限（4KB）

# SetFilePointerEx 的 whence
FILE_BEGIN = 0
FILE_CURRENT = 1
FILE_END = 2

# 访问权限 / 共享模式
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x01
FILE_SHARE_WRITE = 0x02
OPEN_EXISTING = 3

INVALID_HANDLE = ctypes.c_void_p(-1).value

DIGCF_PRESENT = 0x02
DIGCF_DEVICEINTERFACE = 0x10

COMMON_NODES = ("control", "user", "h2c_0", "c2h_0", "h2c_1", "c2h_1", "event_0")


# ==================================================================== 异常
class XdmaError(OSError):
    """XDMA 模块所有异常的基类"""


class XdmaNotFound(XdmaError):
    """没找到设备 / 节点打不开"""


class BarAccessError(XdmaError):
    """BAR 访问越界或出错（越界会蓝屏，所以模块内提前拦住）"""


# ==================================================================== Win32 声明
class GUID(ctypes.Structure):
    _fields_ = [("Data1", ctypes.c_uint32),
                ("Data2", ctypes.c_uint16),
                ("Data3", ctypes.c_uint16),
                ("Data4", ctypes.c_ubyte * 8)]

    def __str__(self):
        d4 = "".join("%02x" % b for b in self.Data4)
        return "{%08x-%04x-%04x-%s-%s}" % (self.Data1, self.Data2, self.Data3,
                                           d4[:4], d4[4:])


class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [("cbSize", wintypes.DWORD),
                ("InterfaceClassGuid", GUID),
                ("Flags", wintypes.DWORD),
                ("Reserved", ctypes.c_size_t)]


# DEFINE_GUID(GUID_DEVINTERFACE_XDMA, 0x74c7e4a9, 0x6d5d, 0x4a70,
#             0xbc,0x0d,0x20,0x69,0x1d,0xff,0x9e,0x9d)
XDMA_GUID = GUID(0x74c7e4a9, 0x6d5d, 0x4a70,
                 (ctypes.c_ubyte * 8)(0xbc, 0x0d, 0x20, 0x69, 0x1d,
                                      0xff, 0x9e, 0x9d))

setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

setupapi.SetupDiGetClassDevsW.restype = ctypes.c_void_p
setupapi.SetupDiGetClassDevsW.argtypes = [ctypes.POINTER(GUID), wintypes.LPCWSTR,
                                           ctypes.c_void_p, wintypes.DWORD]
setupapi.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL
setupapi.SetupDiEnumDeviceInterfaces.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                                 ctypes.POINTER(GUID), wintypes.DWORD,
                                                 ctypes.POINTER(SP_DEVICE_INTERFACE_DATA)]
setupapi.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL
setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [
    ctypes.c_void_p, ctypes.POINTER(SP_DEVICE_INTERFACE_DATA), ctypes.c_void_p,
    wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
setupapi.SetupDiDestroyDeviceInfoList.argtypes = [ctypes.c_void_p]

kernel32.CreateFileW.restype = ctypes.c_void_p
kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                 ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD,
                                 ctypes.c_void_p]
kernel32.SetFilePointerEx.restype = wintypes.BOOL
kernel32.SetFilePointerEx.argtypes = [ctypes.c_void_p, ctypes.c_longlong,
                                      ctypes.POINTER(ctypes.c_longlong), wintypes.DWORD]
kernel32.ReadFile.restype = wintypes.BOOL
kernel32.ReadFile.argtypes = [ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD,
                              ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
kernel32.WriteFile.restype = wintypes.BOOL
kernel32.WriteFile.argtypes = kernel32.ReadFile.argtypes
kernel32.CloseHandle.restype = wintypes.BOOL
kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
kernel32.GetLastError.restype = wintypes.DWORD
kernel32.GetLastError.argtypes = []


# ==================================================================== 底层工具
def _last_error():
    return ctypes.get_last_error()


def _winerr(prefix):
    """把 GetLastError 变成人话"""
    e = _last_error()
    msg = ctypes.FormatError(e).strip() if e else "?"
    return XdmaError("%s (Win32 err=%d: %s)" % (prefix, e, msg))


def is_admin():
    """是否以管理员身份运行（访问 XDMA 通常需要）"""
    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:
        return False


def aligned_buffer(size, align=ALIGN):
    """分配对齐缓冲 —— XDMA 驱动对 DMA 拷贝缓冲的硬性要求

    返回 (持有者, 视图)。持有者必须保留，否则视图会悬空。
    """
    if size <= 0:
        raise ValueError("size 必须为正")
    raw = ctypes.create_string_buffer(size + align)
    addr = (ctypes.addressof(raw) + align - 1) & ~(align - 1)
    return raw, (ctypes.c_char * size).from_address(addr)


def enum_xdma_paths():
    """枚举全部 XDMA 设备接口路径（按 GUID_DEVINTERFACE_XDMA）"""
    h = setupapi.SetupDiGetClassDevsW(ctypes.byref(XDMA_GUID), None, None,
                                      DIGCF_PRESENT | DIGCF_DEVICEINTERFACE)
    if not h or h == INVALID_HANDLE:
        raise _winerr("SetupDiGetClassDevs 失败")
    paths = []
    idx = 0
    try:
        while True:
            did = SP_DEVICE_INTERFACE_DATA()
            did.cbSize = ctypes.sizeof(did)
            if not setupapi.SetupDiEnumDeviceInterfaces(h, None,
                                                        ctypes.byref(XDMA_GUID),
                                                        idx, ctypes.byref(did)):
                break
            need = wintypes.DWORD(0)
            setupapi.SetupDiGetDeviceInterfaceDetailW(h, ctypes.byref(did), None, 0,
                                                      ctypes.byref(need), None)
            if need.value == 0:
                idx += 1
                continue
            buf = ctypes.create_string_buffer(need.value)
            # SP_DEVICE_INTERFACE_DETAIL_DATA_W.cbSize：64 位下含对齐为 8
            ctypes.cast(buf, ctypes.POINTER(wintypes.DWORD))[0] = \
                8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6
            if setupapi.SetupDiGetDeviceInterfaceDetailW(h, ctypes.byref(did), buf,
                                                         need.value, None, None):
                paths.append(ctypes.wstring_at(ctypes.addressof(buf) + 4))
            idx += 1
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(h)
    return paths


def pattern_bytes(n, seed=0x5A5A5A5A):
    """确定性测试图案（每 4 字节一个自增计数，便于定位错位）"""
    n4 = (n + 3) // 4
    buf = bytearray(n4 * 4)
    for i in range(n4):
        v = (seed + i) & 0xFFFFFFFF
        buf[i * 4 + 0] = v & 0xFF
        buf[i * 4 + 1] = (v >> 8) & 0xFF
        buf[i * 4 + 2] = (v >> 16) & 0xFF
        buf[i * 4 + 3] = (v >> 24) & 0xFF
    return bytes(buf[:n])


def _first_diff(a, b):
    """返回 (index, a_val, b_val)，全同返回 None。用于定位数据错位"""
    la, lb = len(a), len(b)
    n = la if la < lb else lb
    for i in range(n):
        if a[i] != b[i]:
            return i, a[i], b[i]
    if la != lb:
        return n, (a[n] if n < la else None), (b[n] if n < lb else None)
    return None


# ==================================================================== 设备
class XdmaDevice(object):
    """一块 XDMA 卡

    用法::

        dev = XdmaDevice()                 # 第一块
        dev = XdmaDevice(index=1)          # 第二块
        dev = XdmaDevice(match="7024")     # 按接口路径关键字挑卡
    """

    def __init__(self, index=0, match=None):
        paths = enum_xdma_paths()
        if not paths:
            raise XdmaNotFound("未找到 XDMA 设备（VEN_10EE）——板子没枚举到？"
                               "驱动装了没？JTAG 下载过 bit 吗？")
        if match:
            hits = [p for p in paths if match.lower() in p.lower()]
            if not hits:
                raise XdmaNotFound("没有接口路径包含 %r，现有：%s" % (match, paths))
            self.path = hits[0]
        else:
            if index >= len(paths):
                raise XdmaNotFound("index %d 越界，共 %d 个 XDMA 设备" % (index, len(paths)))
            self.path = paths[index]
        self.index = paths.index(self.path)
        self.guid = str(XDMA_GUID)
        self._handles = {}
        self._lock = threading.Lock()

    # ---- 基本信息 ----
    def __repr__(self):
        return "<XdmaDevice %d %s>" % (self.index, self.path)

    @staticmethod
    def list_paths():
        """返回所有设备的接口路径字符串列表"""
        return enum_xdma_paths()

    @classmethod
    def list_all(cls):
        """返回所有 XdmaDevice 对象"""
        return [cls(index=i) for i in range(len(enum_xdma_paths()))]

    def node_path(self, node):
        """节点完整路径（字符串），可交给别的工具用"""
        return self.path + "\\" + node

    # ---- 节点管理 ----
    def _open(self, node):
        with self._lock:
            h = self._handles.get(node)
            if h is not None:
                return h
            h = kernel32.CreateFileW(self.node_path(node),
                                     GENERIC_READ | GENERIC_WRITE,
                                     FILE_SHARE_READ | FILE_SHARE_WRITE,
                                     None, OPEN_EXISTING, 0, None)
            if not h or h == INVALID_HANDLE:
                raise XdmaNotFound("打开节点 %r 失败（该节点不存在？当前卡可能"
                                   "是纯回环设计、没有 user/control 节点）：%s"
                                   % (node, self.path))
            self._handles[node] = h
            return h

    def has(self, node):
        """节点是否可用（不抛异常）"""
        try:
            h = self._open(node)
        except XdmaNotFound:
            return False
        return bool(h)

    def nodes(self, probe=None):
        """探测哪些节点存在，返回存在的节点名列表

        probe=None 时探测 COMMON_NODES。
        """
        probe = probe or COMMON_NODES
        found = []
        for n in probe:
            if self.has(n):
                found.append(n)
        return found

    def close(self):
        with self._lock:
            for h in self._handles.values():
                kernel32.CloseHandle(h)
            self._handles.clear()

    def _release(self, node, h):
        """从缓存里摘掉并关闭一个句柄（供 DmaChannel 单独关闭时用，避免双关）"""
        with self._lock:
            if self._handles.get(node) == h:
                del self._handles[node]
                kernel32.CloseHandle(h)
                return True
        return False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- 裸 BAR 访问（带窗口保护）----
    def bar_read(self, node, off, size, window=DEFAULT_WINDOW):
        """在指定节点（BAR）里读 size 字节，带越界保护"""
        if off < 0 or size < 0 or off + size > window:
            raise BarAccessError(
                "拒绝访问 %s 节点偏移 0x%X..0x%X（超出安全窗口 0x%X）。"
                "越界访问 BAR 会导致 0x50 蓝屏！" % (node, off, off + size - 1, window))
        h = self._open(node)
        li = ctypes.c_longlong(off)
        if not kernel32.SetFilePointerEx(h, li, None, FILE_BEGIN):
            raise _winerr("SetFilePointerEx(off=0x%X) 失败" % off)
        raw, buf = aligned_buffer(size)
        n = wintypes.DWORD(0)
        if not kernel32.ReadFile(h, buf, size, ctypes.byref(n), None):
            raise _winerr("ReadFile(%s, 0x%X, %d) 失败" % (node, off, size))
        if n.value != size:
            raise BarAccessError("ReadFile 短读：期望 %d 实际 %d" % (size, n.value))
        return bytes(buf[:size])

    def bar_write(self, node, off, data, window=DEFAULT_WINDOW):
        """往指定节点（BAR）写数据，带越界保护"""
        size = len(data)
        if off < 0 or off + size > window:
            raise BarAccessError(
                "拒绝访问 %s 节点偏移 0x%X..0x%X（超出安全窗口 0x%X）。"
                "越界访问 BAR 会导致 0x50 蓝屏！" % (node, off, off + size - 1, window))
        h = self._open(node)
        li = ctypes.c_longlong(off)
        if not kernel32.SetFilePointerEx(h, li, None, FILE_BEGIN):
            raise _winerr("SetFilePointerEx(off=0x%X) 失败" % off)
        raw, buf = aligned_buffer(size)
        ctypes.memmove(buf, data, size)
        n = wintypes.DWORD(0)
        if not kernel32.WriteFile(h, buf, size, ctypes.byref(n), None):
            raise _winerr("WriteFile(%s, 0x%X, %d) 失败" % (node, off, size))
        if n.value != size:
            raise BarAccessError("WriteFile 短写：期望 %d 实际 %d" % (size, n.value))
        return n.value


# ==================================================================== 寄存器区
class Regs(object):
    """32-bit 寄存器组（默认走 user 节点 = BAR2 / AXI4-Lite Master）

    恒等映射（本工程已实测）：Host 访问 user 偏移 N 字节
    == BD 里 AXI-Lite 地址 N 字节 == 第 N/4 号 32-bit 寄存器。

        with Regs(dev, count=64) as r:
            r.write(0, 0xAC701001)
            assert r.read(0) == 0xAC701001
            print(r.dump())

    安全：window 默认 0x1000，越界抛 BarAccessError（不会蓝屏）。
    """

    def __init__(self, dev, node="user", base=0, count=64,
                 window=DEFAULT_WINDOW, iface_window_check=True):
        self.dev = dev
        self.node = node
        self.base = base
        self.count = count
        self.window = window if iface_window_check else (1 << 32)
        # 预先检查总块是否在窗口内
        end = base + count * 4
        if end > self.window:
            raise BarAccessError(
                "寄存器块 0x%X..0x%X 超出安全窗口 0x%X。"
                "请先用 xci 的 pf0_bar*_aperture_size 静态确认真实映射长度，"
                "再显式传 window=。" % (base, end - 1, self.window))

    # ---- 生命周期 ----
    def close(self):
        self.dev.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- 索引 <-> 偏移 ----
    def offset_of(self, idx):
        if not 0 <= idx < self.count:
            raise IndexError("寄存器索引 %d 越界 (0..%d)" % (idx, self.count - 1))
        return self.base + idx * 4

    # ---- 单个寄存器 ----
    def read(self, idx):
        """读 1 个 32-bit 寄存器"""
        return int.from_bytes(self.dev.bar_read(self.node, self.offset_of(idx), 4,
                                               self.window), "little")

    def write(self, idx, val):
        """写 1 个 32-bit 寄存器"""
        return self.dev.bar_write(self.node, self.offset_of(idx),
                                  (val & 0xFFFFFFFF).to_bytes(4, "little"),
                                  self.window)

    def read_bytes(self, idx, nbytes):
        """从第 idx 号寄存器起读 nbytes 原始字节"""
        return self.dev.bar_read(self.node, self.offset_of(idx), nbytes, self.window)

    def write_bytes(self, idx, blob):
        """从第 idx 号寄存器起写原始字节"""
        return self.dev.bar_write(self.node, self.offset_of(idx), blob, self.window)

    # ---- 64-bit 计数器（lo 在 idx，hi 在 idx+1）----
    def read64(self, idx_lo):
        lo = self.read(idx_lo)
        hi = self.read(idx_lo + 1)
        return (hi << 32) | lo

    def write64(self, idx_lo, val):
        v = val & 0xFFFFFFFFFFFFFFFF
        self.write(idx_lo, v & 0xFFFFFFFF)
        self.write(idx_lo + 1, (v >> 32) & 0xFFFFFFFF)

    # ---- 整块 ----
    def dump(self):
        """一次系统调用读回整个寄存器块（比逐个 read 快 N 倍）"""
        blob = self.dev.bar_read(self.node, self.base, self.count * 4, self.window)
        return [int.from_bytes(blob[i * 4:i * 4 + 4], "little")
                for i in range(self.count)]

    def poke(self, vals):
        """整块写回（dump 的逆操作）"""
        if len(vals) != self.count:
            raise ValueError("需要 %d 个值，收到 %d 个" % (self.count, len(vals)))
        blob = b"".join((v & 0xFFFFFFFF).to_bytes(4, "little") for v in vals)
        return self.dev.bar_write(self.node, self.base, blob, self.window)

    def selftest(self, verbose=False):
        """逐个「写-读-校验」。会覆盖原内容，返回失败索引列表"""
        bad = []
        for i in range(self.count):
            pat = (0x5A5A0000 | i) & 0xFFFFFFFF
            self.write(i, pat)
            got = self.read(i)
            if got != pat:
                bad.append(i)
                if verbose:
                    print("  [%02d] write %08X read %08X  MISMATCH" % (i, pat, got))
            elif verbose:
                print("  [%02d] %08X OK" % (i, pat))
        return bad

    def hexdump(self, vals=None, per_line=4):
        if vals is None:
            vals = self.dump()
        lines = []
        for i in range(0, len(vals), per_line):
            chunk = vals[i:i + per_line]
            cells = "  ".join("[%02d] %08X" % (i + j, v) for j, v in enumerate(chunk))
            lines.append("  off 0x%03X : %s" % (self.base + i * 4, cells))
        return "\n".join(lines)


# ==================================================================== DMA 通道
class DmaChannel(object):
    """描述符 DMA 通道（h2c_0 / c2h_0 / h2c_1 ...）

        with DmaChannel(dev, "h2c_0") as h2c:
            h2c.write(pattern_bytes(1 << 20))     # 1MB 发给 FPGA

    内部自动按 CHUNK=1MB 分块（驱动单笔上限约 1~4MB，超过会挂死）。
    AXI-Stream 模式下 addr 参数会被引擎忽略，这里仍然调用 SetFilePointerEx
    只是为了兼容驱动的接口形态。
    """

    def __init__(self, dev, node="h2c_0", seek=True):
        self.dev = dev
        self.node = node
        self.h = dev._open(node)
        self.seek = seek
        self.total_written = 0
        self.total_read = 0

    def close(self):
        h = self.h
        self.h = None
        if h:
            if not self.dev._release(self.node, h):
                kernel32.CloseHandle(h)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _seek(self, addr):
        # AXI-Stream 模式下驱动忽略地址，seek=False 时直接跳过（避免驱动不支持而报错）
        if not self.seek:
            return
        li = ctypes.c_longlong(addr)
        if not kernel32.SetFilePointerEx(self.h, li, None, FILE_BEGIN):
            raise _winerr("SetFilePointerEx(addr=0x%X) 失败" % addr)

    def write(self, data, addr=0):
        """H2C：把 data 全部写给 FPGA 才返回。返回写入字节数

        !! 性能要点：必须用 ctypes.memmove 填充缓冲。
           写成 `buf[:n] = data[...]`（buf 是 ctypes 字符数组）会逐元素走 Python，
           实测只有 ~17.5M 元素/秒 => H2C 恒为 17 MB/s，与尺寸无关。
           memmove 是 C 级 memcpy，可达 GB/s。
        """
        blob = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        nbytes = len(blob)
        step = nbytes if nbytes < CHUNK else CHUNK
        raw, buf = aligned_buffer(step)        # 复用缓冲，大小贴合实际块
        done = 0
        while done < nbytes:
            n = step if (nbytes - done) > step else (nbytes - done)
            # !! 每块都要显式 seek：驱动【不会】随 WriteFile 自动推进 AXI 地址，
            #    否则多块会全部落在同一地址（后写覆盖前写）。
            self._seek(addr + done)
            ctypes.memmove(buf, blob[done:done + n], n)
            got = wintypes.DWORD(0)
            if not kernel32.WriteFile(self.h, buf, n, ctypes.byref(got), None):
                raise _winerr("WriteFile(%s, %d 字节 @0x%X) 失败" % (self.node, n, done))
            if got.value != n:
                raise XdmaError("WriteFile 短写：期望 %d 实际 %d" % (n, got.value))
            done += n
        self.total_written += done
        return done

    def read(self, size, addr=0):
        """C2H：从 FPGA 读满 size 字节才返回（数据没到时驱动在流控上等待）"""
        out = bytearray(size)                 # 预分配：避免 per-chunk 分配 + 多次拷贝
        step = size if size < CHUNK else CHUNK
        raw, buf = aligned_buffer(step)       # 复用缓冲，大小贴合实际块
        done = 0
        while done < size:
            n = step if (size - done) > step else (size - done)
            self._seek(addr + done)           # 每块显式寻址（驱动不自动推进）
            got = wintypes.DWORD(0)
            if not kernel32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
                raise _winerr("ReadFile(%s, %d 字节 @0x%X) 失败" % (self.node, n, done))
            if got.value == 0:
                raise XdmaError("ReadFile 返回 0 字节（已读 %d / 目标 %d）——"
                                "FPGA 侧没有数据？链路没起来？" % (done, size))
            if done + got.value > size:
                raise XdmaError("ReadFile 超读：已读 %d + 本次 %d > 目标 %d"
                                % (done, got.value, size))
            ctypes.memmove(ctypes.addressof(ctypes.c_char.from_buffer(out, done)),
                           buf, got.value)
            done += got.value
        self.total_read += done
        return bytes(out)

    def read_into(self, size, addr=0):
        """流式接收：每次 yield 一个数据块，避免大块缓冲占内存"""
        got_total = 0
        while got_total < size:
            n = CHUNK if (size - got_total) > CHUNK else (size - got_total)
            self._seek(addr + got_total)    # 每块显式寻址（驱动不自动推进）
            raw, buf = aligned_buffer(n)
            got = wintypes.DWORD(0)
            if not kernel32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
                raise _winerr("ReadFile 失败（已读 %d / 目标 %d）" % (got_total, size))
            if got.value == 0:
                raise XdmaError("ReadFile 返回 0 字节（已读 %d / 目标 %d）"
                                % (got_total, size))
            self.total_read += got.value
            got_total += got.value
            yield bytes(buf[:got.value])

    def zero(self, nbytes, addr=0, block=1 << 20):
        """快速发 nbytes 个零（用 WriteFile 零缓冲，不在 Python 侧拼 bytes）"""
        done = 0
        zeros = bytes(block)
        while done < nbytes:
            n = block if (nbytes - done) > block else (nbytes - done)
            self._seek(addr + done)        # 每块显式寻址（驱动不自动推进）
            raw, buf = aligned_buffer(block)
            if n != block:
                ctypes.memmove(buf, zeros[:n], n)
                src, cnt = buf, n
            else:
                ctypes.memmove(buf, zeros, block)
                src, cnt = buf, block
            got = wintypes.DWORD(0)
            if not kernel32.WriteFile(self.h, src, cnt, ctypes.byref(got), None):
                raise _winerr("WriteFile(zero) 失败 @0x%X" % done)
            done += got.value
        self.total_written += done
        return done


# ==================================================================== 回环测试
class LoopbackResult(object):
    """loopback() 的结果"""

    def __init__(self, size, write_s, read_s, verified, diff, error):
        self.size = size
        self.write_s = write_s
        self.read_s = read_s
        self.verified = verified
        self.diff = diff            # (index, sent, got) 或 None
        self.error = error          # 异常对象或 None

    @property
    def ok(self):
        return self.error is None and self.diff is None

    @property
    def write_bw(self):
        """GB/s"""
        return self.size / self.write_s / 1e9 if self.write_s > 0 else 0.0

    @property
    def read_bw(self):
        """GB/s"""
        return self.size / self.read_s / 1e9 if self.read_s > 0 else 0.0

    def __repr__(self):
        return ("<LoopbackResult %d bytes  H2C %.3f GB/s  C2H %.3f GB/s  "
                "verify=%s%s>" % (self.size, self.write_bw, self.read_bw,
                                  "OK" if self.verified else "FAIL",
                                  "" if self.error is None else "  err=%s" % self.error))


class BandwidthResult(object):
    """bandwidth() 的结果"""

    def __init__(self, size, write_s, read_s, error=None):
        self.size = size
        self.write_s = write_s
        self.read_s = read_s
        self.error = error

    @property
    def ok(self):
        return self.error is None

    @property
    def write_bw(self):
        return self.size / self.write_s / 1e9 if self.write_s > 0 else 0.0

    @property
    def read_bw(self):
        return self.size / self.read_s / 1e9 if self.read_s > 0 else 0.0

    def __repr__(self):
        return ("<BandwidthResult %d bytes  H2C %.3f GB/s (%.1f ms)  "
                "C2H %.3f GB/s (%.1f ms)>"
                % (self.size, self.write_bw, self.write_s * 1e3,
                   self.read_bw, self.read_s * 1e3))


def loopback(dev, size=1 << 20, h2c="h2c_0", c2h="c2h_0", verify=True,
             seed=0x12345678, addr=0, progress=False):
    """FPGA 内 H2C→C2H 直连回环测试（双线程并发）

    这是 AXI-Stream 回环的**唯一正确测法**：C2H 读线程先跑把 tready 拉高，
    H2C 写线程才能推数据；顺序执行必然死锁。

        r = loopback(dev, 1 << 20)     # 1MB
        print(r.ok, r.write_bw, r.read_bw)

    verify=True 时用确定性图案校验数据完整性。
    """
    result = LoopbackResult(size, 0.0, 0.0, False, None, None)
    payload = pattern_bytes(size, seed) if verify else None
    recv = [None]
    err = [None]

    ch_h2c = DmaChannel(dev, h2c)
    ch_c2h = DmaChannel(dev, c2h)

    def _writer():
        try:
            t0 = time.perf_counter()
            ch_h2c.write(payload if verify else bytes(size), addr)
            result.write_s = time.perf_counter() - t0
        except Exception as e:      # noqa: BLE001
            err[0] = e

    def _reader():
        try:
            t0 = time.perf_counter()
            recv[0] = ch_c2h.read(size, addr)
            result.read_s = time.perf_counter() - t0
        except Exception as e:      # noqa: BLE001
            err[0] = e

    tw = threading.Thread(target=_writer, name="xdma-h2c")
    tr = threading.Thread(target=_reader, name="xdma-c2h")
    tr.start()          # 读线程先起（primed 引擎、拉高 tready）
    tw.start()
    if progress:
        print("  回环中 %d 字节 ...（约 %.2f s）" % (size, size / 2e9 * 8))
    tw.join()
    tr.join()
    ch_h2c.close()
    ch_c2h.close()

    result.error = err[0]
    if err[0] is None:
        if verify:
            d = _first_diff(payload, recv[0])
            result.diff = d
            result.verified = d is None
        else:
            result.verified = True
    return result


def bandwidth(dev, size=256 << 20, h2c="h2c_0", c2h="c2h_0", addr=0):
    """并发回环带宽测量（不校验数据，比 loopback 稍快）"""
    return loopback(dev, size=size, h2c=h2c, c2h=c2h, verify=False, addr=addr)


# ==================================================================== AXIS tester
#  配套 FPGA 工程: F:/FPGAPROJ/AC701/XDMA_ST  (ac701_axis_tester.v)
#
#  寄存器 (BAR2 偏移 ≡ AXI 地址, C_PCIEBAR2AXIBAR=0):
#     0x00 CTRL      bit0=SRC_EN bit1=SINK_EN bit2=SOFT_RST(自清)
#     0x04 TX_BYTES_LO   0x08 TX_BYTES_HI
#     0x0C RX_BYTES_LO   0x10 RX_BYTES_HI
#     0x14 RX_ERR        0x18 STATUS      0x1C ID (=0xAC701002)
#
#  与 MM 模式测量的区别: 这里没有 AXI 地址、没有 DDR3 读延迟,
#  source 现造数据 / sink 现吃数据，测的是 PCIe 链路本身的极值。
AXIS_ID = 0xAC701002

_AXIS_CTRL, _AXIS_TX, _AXIS_RX, _AXIS_ERR, _AXIS_STATUS, _AXIS_ID_IDX = 0, 1, 3, 5, 6, 7

_NP = 0xFFFFFFFF


def axis_pattern(nbytes, seed=0):
    """生成 tester 期望的数据流（与 ac701_axis_tester.v 的 source 完全一致）

        每 16B beat:  seq | ~seq | seq^0xDEADBEEF | 0xC0DEC0DE      (小端)
        seq 从 seed 起逐 beat +1，seed 默认 0（sink 的计数值也从 0 开始）
    """
    if nbytes % 16:
        raise ValueError("size 必须是 16 的整数倍（AXIS beat = 128bit）")
    nbeat = nbytes // 16
    if nbeat > (_NP + 1):
        raise ValueError("单个 payload 最多 %d beat（seq 是 32bit）" % (_NP + 1))
    pack = _struct.Struct("<IIII").pack
    return b"".join(pack(s & _NP, (~s) & _NP, (s ^ 0xDEADBEEF) & _NP, 0xC0DEC0DE)
                    for s in range(seed, seed + nbeat))


def axis_check(blob, seed=0, limit=8):
    """校验 tester source 发出的数据。返回错误列表 [(beat, 期望, 实际), ...]"""
    import array as _array
    words = _array.array("I")
    words.frombytes(blob[:len(blob) // 16 * 16])
    if sys.byteorder != "little":
        words.byteswap()
    nbeat = len(words) // 4
    bad = []
    for i in range(nbeat):
        b = i * 4
        s = (seed + i) & _NP
        exp = (s, (~s) & _NP, (s ^ 0xDEADBEEF) & _NP, 0xC0DEC0DE)
        got = (words[b], words[b + 1], words[b + 2], words[b + 3])
        if got != exp:
            bad.append((i, exp, got))
            if len(bad) >= limit:
                break
    return bad


class AxisResult(object):
    """axis_bench() 的结果"""

    def __init__(self, size, mode):
        self.size = size
        self.mode = mode
        self.write_s = 0.0
        self.read_s = 0.0
        self.tx_bytes = 0        # FPGA 侧 source 计数（C2H 发出）
        self.rx_bytes = 0        # FPGA 侧 sink 计数（H2C 收到）
        self.rx_err = 0
        self.bad = []
        self.error = None

    @property
    def ok(self):
        return self.error is None and self.rx_err == 0 and not self.bad

    @property
    def write_bw(self):
        return self.size / self.write_s / 1e9 if self.write_s > 0 else 0.0

    @property
    def read_bw(self):
        return self.size / self.read_s / 1e9 if self.read_s > 0 else 0.0

    @property
    def total_bw(self):
        """全双工聚合（两个方向之和）"""
        return self.write_bw + self.read_bw

    def __repr__(self):
        return ("<AxisResult mode=%s %d bytes  H2C %.3f GB/s  C2H %.3f GB/s  "
                "err=%d%s>" % (self.mode, self.size, self.write_bw, self.read_bw,
                               self.rx_err,
                               "" if self.error is None else "  EXC=%s" % self.error))


def axis_bench(dev, size=64 << 20, mode="both", verify=True,
               h2c="h2c_0", c2h="c2h_0", regs_node="user", progress=False,
               force=False):
    """在 XDMA_ST 工程上测单向 / 全双工带宽

        mode = "h2c"   → 只发不读（sink 单向吞数据，测 PCIe 写上限）
        mode = "c2h"   → 只读不发（source 现造数据，测 PCIe 读上限）
        mode = "both"  → 双向同时跑（测聚合带宽）

    verify=True 时：C2H 回读数据逐 beat 比对；H2C 用 FPGA 侧 sink 的
    RX_ERR / RX_BYTES 判定（不需要把数据读回来）。
    """
    if size % 16:
        raise ValueError("size 必须是 16 的整数倍")
    src_en = mode in ("c2h", "both")
    sink_en = mode in ("h2c", "both")

    r = AxisResult(size, mode)
    payload = axis_pattern(size)
    recv = [None]
    t_w = [0.0]
    t_r = [0.0]
    exc = []

    with Regs(dev, node=regs_node, base=0, count=8) as regs:
        ident = regs.read(_AXIS_ID_IDX)
        if ident != AXIS_ID and not force:
            raise XdmaError(
                "BAR2 0x1C 读到 0x%08X，不是 tester 设计（应为 0x%08X）。\n"
                "  0x00000000 = BRAM 版 / 0xAC701001 = 寄存器版 / 读取失败 = AXI-Lite 未接\n"
                "  如果确认 FPGA 里是别的 AXI-Stream 设计，加 --force 跳过此检查"
                % (ident, AXIS_ID))
        if ident != AXIS_ID:
            print("  !! --force：跳过指纹检查（0x1C = 0x%08X，非 tester）" % ident)

        # 软复位清计数器，再按 mode 置使能
        regs.write(_AXIS_CTRL, 0x4)
        regs.write(_AXIS_CTRL, (0x1 if src_en else 0) | (0x2 if sink_en else 0))

        def _wr():
            try:
                with DmaChannel(dev, h2c, seek=False) as ch:
                    t0 = time.perf_counter()
                    ch.write(payload)
                    t_w[0] = time.perf_counter() - t0
            except Exception as e:      # noqa: BLE001
                exc.append(e)

        def _rd():
            try:
                with DmaChannel(dev, c2h, seek=False) as ch:
                    t0 = time.perf_counter()
                    recv[0] = ch.read(size)
                    t_r[0] = time.perf_counter() - t0
            except Exception as e:      # noqa: BLE001
                exc.append(e)

        # C2H 线程先起来（把 TREADY 拉高），再推 H2C
        t_rd = threading.Thread(target=_rd) if src_en else None
        t_wr = threading.Thread(target=_wr) if sink_en else None
        if progress:
            print("  跑 %s 模式 %d 字节 ..." % (mode, size))
        if t_rd:
            t_rd.start()
        if t_wr:
            t_wr.start()
        if t_rd:
            t_rd.join()
        if t_wr:
            t_wr.join()

        r.write_s = t_w[0]
        r.read_s = t_r[0]
        r.tx_bytes = regs.read64(_AXIS_TX)
        r.rx_bytes = regs.read64(_AXIS_RX)
        r.rx_err = regs.read(_AXIS_ERR)
        regs.write(_AXIS_CTRL, 0x0)      # 收工，关使能

    if exc:
        r.error = exc[0]
        return r

    if verify:
        if mode in ("c2h", "both") and recv[0] is not None:
            r.bad = axis_check(recv[0])
        if sink_en and r.rx_bytes != size:
            r.error = XdmaError("FPGA 侧 sink 只收到 %d / %d 字节 —— "
                                "H2C 没跑完或被提前中止" % (r.rx_bytes, size))
    return r


def loopback_suite(dev, sizes=(1 << 10, 1 << 16, 1 << 20, 8 << 20),
                   h2c="h2c_0", c2h="c2h_0", verbose=True, addr=0):
    """多尺寸扫描，找带宽随尺寸的爬升曲线（识别固定开销 vs 稳态）"""
    rows = []
    for s in sizes:
        r = loopback(dev, s, h2c=h2c, c2h=c2h, verify=(s <= (1 << 20)), addr=addr)
        rows.append(r)
        if verbose:
            print("  %10d B  H2C %6.3f GB/s  C2H %6.3f GB/s  %s"
                  % (s, r.write_bw, r.read_bw,
                     "OK" if r.ok else "FAIL " + str(r.error or r.diff)))
    return rows


# ==================================================================== 工具
def hexdump(blob, base=0, width=16):
    """标准 hexdump 文本"""
    out = []
    for i in range(0, len(blob), width):
        chunk = blob[i:i + width]
        hexs = " ".join("%02X" % b for b in chunk)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append("%08X  %-*s  |%s|" % (base + i, width * 3, hexs, text))
    return "\n".join(out)


def describe():
    """返回驱动能力描述（便于日志留档）"""
    return {
        "module": "xdma.py",
        "version": __version__,
        "python": sys.version.split()[0],
        "guid": str(XDMA_GUID),
        "admin": is_admin(),
        "chunk": CHUNK,
        "align": ALIGN,
        "default_bar_window": DEFAULT_WINDOW,
    }


# ==================================================================== CLI
def _cmd_list(dev):
    print("XDMA GUID : %s" % XDMA_GUID)
    paths = enum_xdma_paths()
    print("设备数量   : %d" % len(paths))
    for i, p in enumerate(paths):
        print("  [%d] %s" % (i, p))
    if not paths:
        return 1
    dev = XdmaDevice()
    found = dev.nodes()
    print("节点探测   : %s" % (", ".join(found) if found else "(无可用节点)"))
    print("管理员权限 : %s" % is_admin())
    dev.close()
    return 0


def _cmd_info(dev):
    print("设备       : %s" % dev)
    found = dev.nodes()
    print("可用节点   : %s" % (", ".join(found) if found else "(无)"))
    for missing in [n for n in COMMON_NODES if n not in found]:
        print("  缺失     : %s" % missing)
    if "user" in found:
        with Regs(dev) as r:
            vals = r.dump()
            print("user 寄存器 (前 %d 个):" % min(r.count, 16))
            print(r.hexdump(vals[:16]))
    dev.close()
    return 0


def _cmd_regs(dev, args):
    with Regs(dev, node=args.node, base=args.base, count=args.nreg) as r:
        if args.dump:
            print(r.hexdump())
            return 0
        if args.selftest:
            print("逐个写-读-校验（会覆盖原内容）...")
            bad = r.selftest(verbose=args.verbose)
            print("结果：%d/%d 通过" % (r.count - len(bad), r.count))
            return 0 if not bad else 3
        if args.idx is None:
            ap.print_help()
            return 1
        if args.val is None:
            v = r.read(args.idx)
            print("read  [%d] @0x%03X = 0x%08X (%d)" % (args.idx, r.offset_of(args.idx), v, v))
            return 0
        r.write(args.idx, args.val)
        back = r.read(args.idx)
        ok = back == (args.val & 0xFFFFFFFF)
        print("write [%d] @0x%03X <= 0x%08X | 读回 0x%08X %s"
              % (args.idx, r.offset_of(args.idx), args.val & 0xFFFFFFFF, back,
                 "OK" if ok else "<<< MISMATCH"))
        return 0 if ok else 3


def _cmd_lb(dev, args):
    sizes = [int(x) for x in args.sizes.split(",")]
    if args.seq:
        rows = _sequential_suite(dev, sizes, args.addr)
    else:
        rows = loopback_suite(dev, sizes=sizes, addr=args.addr)
    bad = [r for r in rows if not r.ok]
    return 0 if not bad else 3


def _sequential_suite(dev, sizes, addr):
    """MM 设计（DDR / BRAM）用：完整写完，再整块读回校验。

    并发模式只对「H2C 直连 C2H」的 AXI-Stream 回环成立；MM 设计里
    读写打同一个 AXI 地址空间，并发会让读线程读到写之前的内容。
    """
    rows = []
    ch_w = DmaChannel(dev, "h2c_0")
    ch_r = DmaChannel(dev, "c2h_0")
    try:
        for s in sizes:
            payload = pattern_bytes(s)
            r = LoopbackResult(s, 0.0, 0.0, False, None, None)
            try:
                t0 = time.perf_counter()
                ch_w.write(payload, addr)
                r.write_s = time.perf_counter() - t0
                t0 = time.perf_counter()
                back = ch_r.read(s, addr)
                r.read_s = time.perf_counter() - t0
                r.diff = _first_diff(payload, back)
                r.verified = r.diff is None
            except Exception as e:      # noqa: BLE001
                r.error = e
            rows.append(r)
            print("  %10d B  H2C %6.3f GB/s  C2H %6.3f GB/s  %s"
                  % (s, r.write_bw, r.read_bw,
                     "OK" if r.ok else "FAIL " + str(r.error or r.diff)))
    finally:
        ch_w.close()
        ch_r.close()
    return rows


def _cmd_bw(dev, args):
    size = args.size
    print("并发回环带宽测量：%d 字节 (%.1f MB) @ AXI 0x%X"
          % (size, size / 1048576.0, args.addr))
    r = bandwidth(dev, size, addr=args.addr)
    if not r.ok:
        print("失败：%s" % r.error)
        return 3
    print("  H2C (host->fpga) : %.3f GB/s  (%.1f ms)" % (r.write_bw, r.write_s * 1e3))
    print("  C2H (fpga->host) : %.3f GB/s  (%.1f ms)" % (r.read_bw, r.read_s * 1e3))
    print("  （Gen2 x4 线速率理论上限约 2.0 GB/s）")
    return 0


def _cmd_axis(dev, args):
    modes = ["h2c", "c2h", "both"] if args.mode == "all" else [args.mode]
    print("AXI-Stream tester 带宽  size=%d (%.1f MB)  校验=%s"
          % (args.size, args.size / 1048576.0, "on" if not args.no_verify else "off"))
    print("  （Gen2 x4 单向线速率 2.00 GB/s，双向各自独立）")
    print("  %-5s %-11s %-11s %-11s %s"
          % ("mode", "H2C GB/s", "C2H GB/s", "聚合 GB/s", "结果"))
    print("  " + "-" * 60)
    worst = 0
    for m in modes:
        r = axis_bench(dev, size=args.size, mode=m,
                       verify=not args.no_verify, progress=False,
                       force=args.force)
        if r.error is not None:
            print("  %-5s 失败：%s" % (m, r.error))
            worst = 3
            continue
        note = "OK"
        if r.bad:
            b = r.bad[0]
            note = "数据错 beat%d 期望%s 实际%s" % b
            worst = 3
        elif r.rx_err:
            note = "sink 报 %d 个 beat 不一致" % r.rx_err
            worst = 3
        print("  %-5s %-11.3f %-11.3f %-11.3f %s"
              % (m,
                 r.write_bw if m != "c2h" else 0.0,
                 r.read_bw if m != "h2c" else 0.0,
                 r.total_bw, note))
        if m == "both" and not args.no_verify:
            print("        FPGA 计数: TX=%d B RX=%d B ERR=%d"
                  % (r.tx_bytes, r.rx_bytes, r.rx_err))
    return worst


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="xdma",
        description="Xilinx XDMA 驱动 Python 封装（寄存器 + DMA + 回环带宽）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="安全提醒：BAR 访问被限制在 0x0~0x%X，越界会蓝屏。" % DEFAULT_WINDOW)
    ap.add_argument("--version", action="version", version="xdma.py " + __version__)
    ap.add_argument("-d", "--device", type=int, default=0, help="第几块 XDMA 卡（默认 0）")
    ap.add_argument("--match", help="按接口路径关键字选卡，如 7024")
    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("list", help="枚举设备与节点")
    sub.add_parser("info", help="节点与寄存器概览")

    p = sub.add_parser("regs", help="读写 user 寄存器（可指定 --node）")
    p.add_argument("idx", nargs="?", type=int, help="寄存器索引（0-based）")
    p.add_argument("val", nargs="?", type=lambda s: int(s, 0), help="写入值")
    p.add_argument("--node", default="user", help="节点名（默认 user）")
    p.add_argument("--base", type=lambda s: int(s, 0), default=0, help="寄存器块基址")
    p.add_argument("--nreg", type=int, default=64, help="寄存器个数（默认 64）")
    p.add_argument("--dump", action="store_true", help="读回全部寄存器")
    p.add_argument("--selftest", action="store_true", help="逐个写-读-校验")
    p.add_argument("-v", "--verbose", action="store_true")

    p = sub.add_parser("lb", help="回环 + 数据完整性校验（多尺寸）")
    p.add_argument("--sizes", default="1024,65536,1048576,8388608",
                   help="逗号分隔的尺寸列表")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0,
                   help="AXI 基址（MM 设计必须填 DDR 段，如 0x80000000；"
                        "AXI-Stream 纯回环填 0）")
    p.add_argument("--seq", action="store_true",
                   help="顺序模式（写完再读）。MM 设计必加；不加则用并发回环模式")

    p = sub.add_parser("bw", help="并发回环带宽测量")
    p.add_argument("--size", type=lambda s: int(s, 0), default=256 << 20,
                   help="总字节数（默认 256MB）")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0,
                   help="AXI 基址（MM 设计必须填 DDR 段，如 0x80000000）")

    p = sub.add_parser("axis", help="AXI-Stream tester 单向/全双工带宽（XDMA_ST 工程）")
    p.add_argument("--size", type=lambda s: int(s, 0), default=64 << 20,
                   help="单次传输字节数，须为 16 的倍数（默认 64MB）")
    p.add_argument("--mode", choices=["both", "h2c", "c2h", "all"], default="all",
                   help="both=双向同时 / h2c=只发 / c2h=只读 / all=三种都跑（默认）")
    p.add_argument("--no-verify", action="store_true", help="跳过数据校验（更快）")
    p.add_argument("--force", action="store_true",
                   help="跳过 BAR2 0x1C 指纹检查（用于非 tester 的 AXI-Stream 设计）")

    args = ap.parse_args(argv)
    if not args.cmd:
        ap.print_help()
        return 1

    try:
        dev = XdmaDevice(index=args.device, match=args.match)
    except XdmaError as e:
        print("错误：%s" % e)
        return 2

    try:
        if args.cmd == "list":
            return _cmd_list(dev)
        if args.cmd == "info":
            return _cmd_info(dev)
        if args.cmd == "regs":
            return _cmd_regs(dev, args)
        if args.cmd == "lb":
            return _cmd_lb(dev, args)
        if args.cmd == "bw":
            return _cmd_bw(dev, args)
        if args.cmd == "axis":
            return _cmd_axis(dev, args)
    except XdmaError as e:
        print("错误：%s" % e)
        return 2
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())