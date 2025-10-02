#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J2534 SocketCAN Suite - Improved Core
- Correct J2534 prototypes (stdcall) per v04.04
- Robust connect + bitrate set
- Optional hardware RX filter (PassThruStartMsgFilter)
- Fixed return codes and error text mapping
- Safe reader thread + UI pump, candump-style logger
Target: 32-bit Python on Windows with 32-bit J2534 DLL
"""
import os, sys, threading, time, queue, random, re, json, ctypes as ct
from ctypes import wintypes as w
from datetime import datetime
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import winreg

# ===========================
# J2534 constants (v04.04)
# ===========================
DATA_BUFFER_SIZE = 4128

# Protocol IDs
J1850PWM=1; J1850VPW=2; ISO9141=3; ISO14230=4; CAN=5; ISO15765=6

# Flags (TxFlags)
TX_FLAG_STANDARD_CAN  = 0x00000000
TX_FLAG_EXTENDED_CAN  = 0x00000001

# IOCTL IDs
GET_CONFIG = 0x01
SET_CONFIG = 0x02
READ_VBATT = 0x03
CLEAR_TX_BUFFER = 0x07
CLEAR_RX_BUFFER = 0x08
CLEAR_MSG_FILTERS = 0x0A

# Config Parameters
DATA_RATE = 0x01
LOOPBACK  = 0x03

# Filters
PASS_FILTER = 0x00000001
BLOCK_FILTER= 0x00000002
FLOW_CONTROL_FILTER = 0x00000003

# Status / Return codes (subset)
STATUS_NOERROR       = 0x00
ERR_NOT_SUPPORTED    = 0x01
ERR_INVALID_CHANNEL_ID = 0x02
ERR_INVALID_PROTOCOL_ID= 0x03
ERR_NULL_PARAMETER   = 0x04
ERR_INVALID_IOCTL_ID = 0x05
ERR_INVALID_FLAGS    = 0x06
ERR_FAILED           = 0x07  # note: spec defines 0x07 as generic failure; vendors differ on BUFFER_EMPTY
ERR_DEVICE_NOT_CONNECTED = 0x08
ERR_TIMEOUT          = 0x0A
ERR_BUFFER_EMPTY     = 0x07  # seen in many vendor headers; tolerance in code below

ERR_TEXT = {
    STATUS_NOERROR: "OK",
    ERR_NOT_SUPPORTED: "ERR_NOT_SUPPORTED",
    ERR_INVALID_CHANNEL_ID: "ERR_INVALID_CHANNEL_ID",
    ERR_INVALID_PROTOCOL_ID: "ERR_INVALID_PROTOCOL_ID",
    ERR_NULL_PARAMETER: "ERR_NULL_PARAMETER",
    ERR_INVALID_IOCTL_ID: "ERR_INVALID_IOCTL_ID",
    ERR_INVALID_FLAGS: "ERR_INVALID_FLAGS",
    ERR_FAILED: "ERR_FAILED/BUFFER_EMPTY (vendor-specific overlap)",
    ERR_DEVICE_NOT_CONNECTED: "ERR_DEVICE_NOT_CONNECTED",
    ERR_TIMEOUT: "ERR_TIMEOUT",
}

# ===========================
# J2534 structures
# ===========================
class SCONFIG(ct.Structure):
    _fields_ = [('Parameter', w.DWORD), ('Value', w.DWORD)]

class SCONFIG_LIST(ct.Structure):
    _fields_ = [('NumOfParams', w.ULONG), ('ConfigPtr', ct.POINTER(SCONFIG))]

class PASSTHRU_MSG(ct.Structure):
    _fields_ = [
        ('ProtocolID', w.ULONG),
        ('RxStatus',   w.ULONG),
        ('TxFlags',    w.ULONG),
        ('Timestamp',  w.ULONG),
        ('DataSize',   w.ULONG),
        ('ExtraDataIndex', w.ULONG),
        ('Data',       ct.c_ubyte * DATA_BUFFER_SIZE),
    ]

class SBYTE_ARRAY(ct.Structure):
    _fields_ = [('NumOfBytes', w.ULONG),
                ('BytePtr',    ct.POINTER(ct.c_ubyte))]

# ===========================
# Prototypes (stdcall)
# ===========================
# LONG PassThruOpen(char* pName, unsigned long* pDeviceID)
PTOPEN       = ct.WINFUNCTYPE(w.ULONG, ct.c_char_p, ct.POINTER(w.ULONG))
# LONG PassThruClose(unsigned long DeviceID)
PTCLOSE      = ct.WINFUNCTYPE(w.ULONG, w.ULONG)
# LONG PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags, unsigned long Baud, unsigned long* pChannelID)
PTCONNECT    = ct.WINFUNCTYPE(w.ULONG, w.ULONG, w.ULONG, w.ULONG, w.ULONG, ct.POINTER(w.ULONG))
# LONG PassThruDisconnect(unsigned long ChannelID)
PTDISCON     = ct.WINFUNCTYPE(w.ULONG, w.ULONG)
# LONG PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout)
PTREADMSGS   = ct.WINFUNCTYPE(w.ULONG, w.ULONG, ct.POINTER(PASSTHRU_MSG), ct.POINTER(w.ULONG), w.ULONG)
# LONG PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG* pMsg, unsigned long* pNumMsgs, unsigned long Timeout)
PTWRITEMSGS  = ct.WINFUNCTYPE(w.ULONG, w.ULONG, ct.POINTER(PASSTHRU_MSG), ct.POINTER(w.ULONG), w.ULONG)
# LONG PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG* pMaskMsg, PASSTHRU_MSG* pPatternMsg, PASSTHRU_MSG* pFlowControlMsg, unsigned long* pFilterID)
PTSTARTFLT   = ct.WINFUNCTYPE(w.ULONG, w.ULONG, w.ULONG, ct.POINTER(PASSTHRU_MSG), ct.POINTER(PASSTHRU_MSG), ct.POINTER(PASSTHRU_MSG), ct.POINTER(w.ULONG))
# LONG PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID)
PTSTOPFLT    = ct.WINFUNCTYPE(w.ULONG, w.ULONG, w.ULONG)
# LONG PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void* pInput, void* pOutput)
PTIOCTL      = ct.WINFUNCTYPE(w.ULONG, w.ULONG, w.ULONG, ct.c_void_p, ct.c_void_p)

# ===========================
# Device discovery
# ===========================
class J2534Device:
    def __init__(self, name, vendor, function_lib):
        self.name = name
        self.vendor = vendor
        self.function_lib = function_lib
    def __str__(self):
        return f"{self.name} ({self.vendor})"

def enumerate_j2534_devices():
    devices = []
    key_path = r"SOFTWARE\PassThruSupport.04.04"
    for view in (winreg.KEY_WOW64_32KEY, winreg.KEY_WOW64_64KEY):
        try:
            base = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path, 0, winreg.KEY_READ | view)
        except OSError:
            continue
        i = 0
        while True:
            try:
                subname = winreg.EnumKey(base, i)
            except OSError:
                break
            i += 1
            try:
                sub = winreg.OpenKey(base, subname)
                vendor = winreg.QueryValueEx(sub, "Vendor")[0]
                dll = winreg.QueryValueEx(sub, "FunctionLibrary")[0]
                dev = J2534Device(subname, vendor, dll)
                if not any(d.function_lib.lower() == dll.lower() for d in devices):
                    devices.append(dev)
                winreg.CloseKey(sub)
            except OSError:
                pass
        winreg.CloseKey(base)
    return devices

# ===========================
# J2534 wrapper
# ===========================
class J2534:
    def __init__(self, dll_path: str):
        self.dll = ct.windll.LoadLibrary(dll_path)
        self.PassThruOpen       = PTOPEN(     ('PassThruOpen',       self.dll))
        self.PassThruClose      = PTCLOSE(    ('PassThruClose',      self.dll))
        self.PassThruConnect    = PTCONNECT(  ('PassThruConnect',    self.dll))
        self.PassThruDisconnect = PTDISCON(   ('PassThruDisconnect', self.dll))
        self.PassThruReadMsgs   = PTREADMSGS( ('PassThruReadMsgs',   self.dll))
        self.PassThruWriteMsgs  = PTWRITEMSGS(('PassThruWriteMsgs',  self.dll))
        self.PassThruStartMsgFilter = PTSTARTFLT(('PassThruStartMsgFilter', self.dll))
        self.PassThruStopMsgFilter  = PTSTOPFLT (('PassThruStopMsgFilter',  self.dll))
        self.PassThruIoctl      = PTIOCTL(    ('PassThruIoctl',      self.dll))

        self.device_id  = w.ULONG(0)
        self.channel_id = w.ULONG(0)
        self.active_filters = []

    def _err(self, code:int, ctx:str):
        txt = ERR_TEXT.get(code, f"0x{code:08X}")
        return RuntimeError(f"{ctx} failed: {txt}")

    def open(self, name: str = None):
        pname = name.encode('ascii') if name else None
        r = self.PassThruOpen(pname, ct.byref(self.device_id))
        if r != STATUS_NOERROR:
            raise self._err(r, "PassThruOpen")
        return self.device_id.value

    def close(self):
        # best-effort cleanup
        if self.channel_id.value:
            try: self.PassThruDisconnect(self.channel_id)
            except Exception: pass
            self.channel_id = w.ULONG(0)
        if self.device_id.value:
            try: self.PassThruClose(self.device_id)
            except Exception: pass
            self.device_id = w.ULONG(0)

    def connect_can(self, bitrate: int, use_iso15765: bool = False, extended_ids: bool = False):
        chan = w.ULONG(0)
        proto = ISO15765 if use_iso15765 else CAN
        flags = 0x00000100 if extended_ids else 0x00000000  # some stacks accept 0x100 as 29-bit flag at connect

        attempts = [
            (proto, flags, 0),
            (proto, 0,     0),
            (CAN,   flags, bitrate),
            (CAN,   0,     bitrate),
            (ISO15765, flags, bitrate),
        ]
        last = None
        for pr, fl, bd in attempts:
            r = self.PassThruConnect(self.device_id, pr, fl, bd, ct.byref(chan))
            if r == STATUS_NOERROR:
                self.channel_id = chan
                break
            last = r
        if not self.channel_id.value:
            raise self._err(last or ERR_FAILED, "PassThruConnect")

        # cleanup buffers; ignore failures
        try:
            self.PassThruIoctl(self.channel_id, CLEAR_RX_BUFFER, None, None)
            self.PassThruIoctl(self.channel_id, CLEAR_TX_BUFFER, None, None)
            self.PassThruIoctl(self.channel_id, CLEAR_MSG_FILTERS, None, None)
        except Exception:
            pass

        # set bitrate via IOCTL if baud 0 path used
        self._set_bitrate(bitrate)
        return self.channel_id.value

    def _set_bitrate(self, bitrate:int):
        # try bps then kbps
        cfg = (SCONFIG * 2)()
        cfg[0].Parameter = DATA_RATE
        cfg[1].Parameter = LOOPBACK
        cfg[1].Value = 0
        for attempt in (int(bitrate), int(bitrate//1000)):
            cfg[0].Value = attempt
            cfglist = SCONFIG_LIST(2, ct.cast(cfg, ct.POINTER(SCONFIG)))
            r = self.PassThruIoctl(self.channel_id, SET_CONFIG, ct.byref(cfglist), None)
            if r == STATUS_NOERROR:
                return
        # not fatal: some tools fix bitrate by network

    def start_pass_filter(self, base_id:int, mask:int, extended:bool=False):
        """Hardware RX filter: pass frames where (ID & mask) == (base & mask)"""
        filt_id = w.ULONG(0)
        def mkmsg(can_id:int):
            m = PASSTHRU_MSG()
            m.ProtocolID = CAN
            m.TxFlags = TX_FLAG_EXTENDED_CAN if (extended or can_id > 0x7FF) else TX_FLAG_STANDARD_CAN
            arb = can_id & (0x1FFFFFFF if (m.TxFlags & TX_FLAG_EXTENDED_CAN) else 0x7FF)
            id_bytes = arb.to_bytes(4, 'little', signed=False)
            for i,b in enumerate(id_bytes): m.Data[i] = b
            m.DataSize = 4
            return m

        mask_msg    = mkmsg(mask)
        pattern_msg = mkmsg(base_id)
        r = self.PassThruStartMsgFilter(self.channel_id, PASS_FILTER,
                                        ct.byref(mask_msg), ct.byref(pattern_msg), None, ct.byref(filt_id))
        if r != STATUS_NOERROR:
            raise self._err(r, "PassThruStartMsgFilter")
        self.active_filters.append(filt_id.value)
        return filt_id.value

    def stop_filter(self, filt_id:int):
        try:
            self.PassThruStopMsgFilter(self.channel_id, w.ULONG(filt_id))
            self.active_filters = [f for f in self.active_filters if f != filt_id]
        except Exception:
            pass

    def write_can(self, can_id: int, data: bytes, extended: bool = False, timeout_ms:int=100):
        msg = PASSTHRU_MSG()
        msg.ProtocolID = CAN
        msg.TxFlags    = TX_FLAG_EXTENDED_CAN if (extended or can_id>0x7FF) else TX_FLAG_STANDARD_CAN
        arb = can_id & (0x1FFFFFFF if (msg.TxFlags & TX_FLAG_EXTENDED_CAN) else 0x7FF)
        id_bytes = arb.to_bytes(4, 'little', signed=False)
        payload  = id_bytes + data
        if len(payload) > DATA_BUFFER_SIZE:
            raise ValueError("Frame too long")
        for i,b in enumerate(payload): msg.Data[i] = b
        msg.DataSize = len(payload)

        num = w.ULONG(1)
        r = self.PassThruWriteMsgs(self.channel_id, ct.byref(msg), ct.byref(num), w.ULONG(timeout_ms))
        if r != STATUS_NOERROR:
            raise self._err(r, "PassThruWriteMsgs")
        if num.value != 1:
            raise RuntimeError(f"Partial write: {num.value}/1")
        return True

    def read_can_batch(self, max_msgs=64, timeout_ms=50):
        arr = (PASSTHRU_MSG * max_msgs)()
        n = w.ULONG(max_msgs)
        r = self.PassThruReadMsgs(self.channel_id, arr, ct.byref(n), w.ULONG(timeout_ms))
        frames = []
        if r in (STATUS_NOERROR, ERR_TIMEOUT, ERR_BUFFER_EMPTY):
            for i in range(n.value):
                msg = arr[i]
                if msg.DataSize < 4:
                    continue
                arb = int.from_bytes(bytes(msg.Data[0:4]), 'little')
                extended = (arb > 0x7FF)
                dlc = min(msg.DataSize - 4, 8)
                data = bytes(msg.Data[4:4+dlc])
                ts = msg.Timestamp
                frames.append((arb, extended, data, ts))
        else:
            # don’t hard fail on transient vendor “buffer empty” codes
            pass
        return frames

# ===========================
# Utilities
# ===========================
HEXD = re.compile(r'^[0-9A-Fa-f]+$')

def parse_cansend(arg:str):
    if '#' not in arg:
        raise ValueError("Format: ID#DATA")
    sid, sdata = arg.split('#', 1)
    extended = len(sid) > 3
    if not HEXD.match(sid):
        raise ValueError("ID must be hex")
    can_id = int(sid, 16)
    if sdata.upper().startswith('R'):
        raise ValueError("RTR not implemented")
    if not HEXD.match(sdata) or len(sdata) % 2:
        raise ValueError("DATA must be even-length hex")
    data = bytes.fromhex(sdata)
    if len(data) > 8:
        raise ValueError("Max 8 bytes for classic CAN")
    return can_id, extended, data

def fmt_frame(can_id, ext, data, ts=None):
    idfmt = f"{can_id:08X}" if ext else f"{can_id:03X}"
    d = ''.join(f"{b:02X}" for b in data)
    if ts is None:
        return f"{idfmt}#{d}"
    return f"({ts:10d}) {idfmt}#{d}"

def parse_id_mask(token:str):
    if ':' not in token: return None
    sid, smask = token.split(':',1)
    if not HEXD.match(sid) or not HEXD.match(smask): return None
    return (int(sid,16), int(smask,16))

def filter_match(can_id:int, filt):
    if not filt: return True
    base, mask = filt
    return (can_id & mask) == (base & mask)

# ===========================
# GUI Application
# ===========================
class CANUtilitySuite(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("J2534 SocketCAN Suite")
        self.geometry("1400x900")
        # State
        self.j2534 = None
        self.available_devices = []
        self.rx_thread = None
        self.rx_running = threading.Event()
        self.rx_queue = queue.Queue(maxsize=20000)
        self.current_mode = tk.StringVar(self, value="idle")
        self.dump_filter = None
        self.sniffer_last = {}
        self.generators = []
        self.stats = {'rx_total':0,'tx_total':0,'rx_errors':0,'current_rate':0}
        self.log_file = None
        self._build_ui()
        self._scan_devices()
        self._start_gui_pump()

    # ---------- UI build ----------
    def _build_ui(self):
        nb = ttk.Notebook(self); nb.pack(fill='both', expand=True, padx=6, pady=6)
        self._tab_conn(nb); self._tab_monitor(nb); self._tab_send(nb)
        self._tab_gen(nb); self._tab_sniffer(nb); self._tab_player(nb); self._tab_stats(nb)
        sb = ttk.Frame(self); sb.pack(fill='x', side='bottom', padx=6, pady=2)
        self.status_label = ttk.Label(sb, text="● Disconnected", foreground="red"); self.status_label.pack(side='left')
        self.stats_label = ttk.Label(sb, text="RX:0 | TX:0 | Err:0 | Rate:0 fps"); self.stats_label.pack(side='right')

    def _tab_conn(self, nb):
        f = ttk.Frame(nb); nb.add(f, text='⚙ Connection')
        lf = ttk.LabelFrame(f, text="J2534 Device", padding=10); lf.pack(fill='x', padx=8, pady=8)
        ttk.Label(lf, text="Device:").grid(row=0,column=0,sticky='w')
        self.device_combo = ttk.Combobox(lf, state='readonly', width=60); self.device_combo.grid(row=0,column=1,sticky='ew',padx=5)
        self.device_combo.bind('<<ComboboxSelected>>', self._on_dev_sel)
        ttk.Button(lf, text="🔄 Refresh", command=self._scan_devices).grid(row=0,column=2,padx=5)
        ttk.Label(lf, text="DLL Path:").grid(row=1,column=0,sticky='w')
        self.dll_path = tk.StringVar(self); ttk.Entry(lf, textvariable=self.dll_path, state='readonly', width=70).grid(row=1,column=1,columnspan=2,sticky='ew',padx=5)
        lf.columnconfigure(1, weight=1)

        cf = ttk.LabelFrame(f, text="CAN Config", padding=10); cf.pack(fill='x', padx=8, pady=8)
        ttk.Label(cf, text="Bitrate:").grid(row=0,column=0,sticky='w'); self.bitrate=tk.StringVar(self, value="500000")
        ttk.Combobox(cf, textvariable=self.bitrate, values=["125000","250000","500000","1000000"], width=12).grid(row=0,column=1,sticky='w',padx=5)
        self.chk_iso = tk.BooleanVar(self, value=False)
        self.chk_ext = tk.BooleanVar(self, value=False)
        ttk.Checkbutton(cf, text="Use ISO15765", variable=self.chk_iso).grid(row=0,column=2, padx=15, sticky='w')
        ttk.Checkbutton(cf, text="29-bit IDs",   variable=self.chk_ext).grid(row=0,column=3, sticky='w')

        bf = ttk.Frame(f); bf.pack(fill='x', padx=8, pady=10)
        self.connect_btn = ttk.Button(bf, text="🔌 Connect", command=self._connect, width=20); self.connect_btn.pack(side='left', padx=5)
        self.disconnect_btn = ttk.Button(bf, text="⏏ Disconnect", command=self._disconnect, state='disabled', width=20); self.disconnect_btn.pack(side='left', padx=5)

        info = ttk.LabelFrame(f, text="Device Info / Notes", padding=10); info.pack(fill='both', expand=True, padx=8, pady=8)
        self.info_text = scrolledtext.ScrolledText(info, height=12, font=('Consolas', 9)); self.info_text.pack(fill='both', expand=True)
        self.info_text.insert('1.0', "No device selected.\n")

    def _tab_monitor(self, nb):
        f = ttk.Frame(nb); nb.add(f, text='📊 Monitor (candump)')
        ctrl = ttk.Frame(f); ctrl.pack(fill='x', padx=8, pady=8)
        ttk.Label(ctrl, text="Filter (ID:MASK):").pack(side='left'); self.monitor_filter=tk.StringVar(self)
        ttk.Entry(ctrl, textvariable=self.monitor_filter, width=20).pack(side='left', padx=6)
        ttk.Button(ctrl, text="▶ Start", command=self._start_monitor).pack(side='left', padx=4)
        ttk.Button(ctrl, text="⏹ Stop", command=self._stop_monitor).pack(side='left', padx=4)
        ttk.Button(ctrl, text="🗑 Clear", command=lambda:self._clear_text(self.monitor_text)).pack(side='left', padx=10)
        self.monitor_pause=tk.BooleanVar(self, value=False); ttk.Checkbutton(ctrl, text="⏸ Pause", variable=self.monitor_pause).pack(side='left', padx=8)
        ttk.Button(ctrl, text="💾 Save Log…", command=self._choose_log).pack(side='right')
        self.monitor_text = scrolledtext.ScrolledText(f, height=35, font=('Consolas', 9),
                                                      bg='#1e1e1e', fg='#00ff00', insertbackground='white')
        self.monitor_text.pack(fill='both', expand=True, padx=8, pady=6)

    def _tab_send(self, nb):
        f=ttk.Frame(nb); nb.add(f, text='📤 Send (cansend)')
        qf=ttk.LabelFrame(f, text="Quick Send", padding=10); qf.pack(fill='x', padx=8, pady=8)
        ttk.Label(qf, text="ID#DATA (e.g. 123#DEADBEEF)").grid(row=0,column=0,sticky='w',columnspan=2)
        self.send_input=tk.StringVar(self); ent=ttk.Entry(qf,textvariable=self.send_input,width=48,font=('Consolas',11))
        ent.grid(row=1,column=0,sticky='ew',padx=5,pady=6); ent.bind('<Return>', lambda e:self._send_quick())
        ttk.Button(qf, text="📤 Send", command=self._send_quick, width=16).grid(row=1,column=1,padx=5)

        bf=ttk.LabelFrame(f, text="Builder", padding=10); bf.pack(fill='x', padx=8, pady=8)
        self.builder_id=tk.StringVar(self, value="123"); self.builder_ext=tk.BooleanVar(self, value=False); self.builder_data=tk.StringVar(self)
        ttk.Label(bf,text="CAN ID (hex):").grid(row=0,column=0,sticky='w'); ttk.Entry(bf,textvariable=self.builder_id,width=14).grid(row=0,column=1,sticky='w',padx=5)
        ttk.Checkbutton(bf,text="Extended (29-bit)",variable=self.builder_ext).grid(row=0,column=2,sticky='w',padx=12)
        ttk.Label(bf,text="Data (hex):").grid(row=1,column=0,sticky='w'); ttk.Entry(bf,textvariable=self.builder_data,width=32).grid(row=1,column=1,sticky='w',padx=5)
        ttk.Button(bf, text="📤 Send Built", command=self._send_built, width=16).grid(row=1,column=2,padx=8)

        rf=ttk.LabelFrame(f, text="Recent TX", padding=10); rf.pack(fill='both', expand=True, padx=8, pady=8)
        self.tx_text=scrolledtext.ScrolledText(rf, height=14, font=('Consolas',9)); self.tx_text.pack(fill='both', expand=True)

    def _tab_gen(self, nb):
        f=ttk.Frame(nb); nb.add(f, text='🔄 Generator (cangen)')
        cf=ttk.LabelFrame(f,text="Config",padding=10); cf.pack(fill='x', padx=8,pady=8)
        ttk.Label(cf,text="CAN ID(s):").grid(row=0,column=0,sticky='w'); self.gen_id=tk.StringVar(self, value="123")
        ttk.Entry(cf,textvariable=self.gen_id,width=18).grid(row=0,column=1,sticky='w',padx=5)
        ttk.Label(cf,text="(single or start:end)").grid(row=0,column=2,sticky='w')
        ttk.Label(cf,text="Interval (ms):").grid(row=1,column=0,sticky='w'); self.gen_interval=tk.StringVar(self, value="100")
        ttk.Entry(cf,textvariable=self.gen_interval,width=10).grid(row=1,column=1,sticky='w',padx=5)
        ttk.Label(cf,text="DLC:").grid(row=2,column=0,sticky='w'); self.gen_dlc=tk.StringVar(self, value="8")
        ttk.Combobox(cf,textvariable=self.gen_dlc,values=list(range(9)),width=8).grid(row=2,column=1,sticky='w',padx=5)
        self.gen_random_data=tk.BooleanVar(self, value=True); ttk.Checkbutton(cf,text="Random Data",variable=self.gen_random_data).grid(row=3,column=0,sticky='w')
        self.gen_random_id=tk.BooleanVar(self, value=False); ttk.Checkbutton(cf,text="Random ID",variable=self.gen_random_id).grid(row=3,column=1,sticky='w')
        ttk.Button(cf,text="▶ Start",command=self._start_gen,width=16).grid(row=4,column=0,pady=8)
        ttk.Button(cf,text="⏹ Stop All",command=self._stop_all_gen,width=16).grid(row=4,column=1,pady=8)
        af=ttk.LabelFrame(f,text="Active Generators",padding=10); af.pack(fill='both',expand=True,padx=8,pady=8)
        self.gen_list=scrolledtext.ScrolledText(af,height=18,font=('Consolas',9)); self.gen_list.pack(fill='both',expand=True)

    def _tab_sniffer(self, nb):
        f=ttk.Frame(nb); nb.add(f, text='🔍 Sniffer (cansniffer)')
        ctrl=ttk.Frame(f); ctrl.pack(fill='x', padx=8,pady=8)
        ttk.Label(ctrl,text="Filter (ID:MASK):").pack(side='left'); self.sniffer_filter=tk.StringVar(self)
        ttk.Entry(ctrl,textvariable=self.sniffer_filter,width=20).pack(side='left',padx=6)
        ttk.Button(ctrl,text="▶ Start",command=self._start_sniffer).pack(side='left',padx=4)
        ttk.Button(ctrl,text="⏹ Stop",command=self._stop_sniffer).pack(side='left',padx=4)
        ttk.Button(ctrl,text="🗑 Clear",command=lambda:self._clear_text(self.sniffer_text)).pack(side='left',padx=10)
        self.sniffer_text=scrolledtext.ScrolledText(f,height=35,font=('Consolas',9),
                                                   bg='#1e1e1e',fg='#ffff00',insertbackground='white')
        self.sniffer_text.pack(fill='both',expand=True,padx=8,pady=6)

    def _tab_player(self, nb):
        f=ttk.Frame(nb); nb.add(f, text='▶ Player (canplayer)')
        self.player_status = tk.StringVar(self, value="Load a candump log to enable playback.")
        ttk.Label(f, textvariable=self.player_status).pack(anchor='w', padx=8, pady=8)
        self.playback_frames=[]; self.playback_thread=None
        self.playback_running=threading.Event(); self.playback_paused=threading.Event()
        bar=ttk.Frame(f); bar.pack(fill='x', padx=8, pady=6)
        ttk.Button(bar,text="Load Log…", command=self._load_log_for_playback).pack(side='left')
        self.player_speed=tk.DoubleVar(self, value=1.0); ttk.Label(bar,text="Speed").pack(side='left',padx=8)
        ttk.Scale(bar,from_=0.1,to=5.0,variable=self.player_speed,orient='horizontal',length=200).pack(side='left')
        ttk.Button(bar,text="▶ Play", command=self._start_playback).pack(side='left',padx=6)
        ttk.Button(bar,text="⏹ Stop", command=self._stop_playback).pack(side='left')

    def _tab_stats(self, nb):
        f=ttk.Frame(nb); nb.add(f, text='📈 Statistics')
        grid=ttk.Frame(f); grid.pack(anchor='w', padx=10, pady=10)
        self._statlbl={}
        for i,(lab,key) in enumerate([("Total RX","rx_total"),("Total TX","tx_total"),("RX Errors","rx_errors"),("Rate (fps)","current_rate")]):
            ttk.Label(grid,text=lab+":",font=('TkDefaultFont',10,'bold')).grid(row=i,column=0,sticky='w',padx=6,pady=4)
            self._statlbl[key]=ttk.Label(grid,text="0"); self._statlbl[key].grid(row=i,column=1,sticky='w',padx=6,pady=4)

    # ---------- device ----------
    def _scan_devices(self):
        devs = enumerate_j2534_devices()
        self.available_devices = devs
        if devs:
            self.device_combo['values']=[str(d) for d in devs]
            self.device_combo.current(0)
            self.dll_path.set(devs[0].function_lib)
            self._update_info()
        else:
            messagebox.showwarning("No Devices", "No J2534 devices found. Install drivers.")

    def _on_dev_sel(self, _evt=None):
        idx = self.device_combo.current()
        if 0 <= idx < len(self.available_devices):
            self.dll_path.set(self.available_devices[idx].function_lib)
            self._update_info()

    def _update_info(self):
        idx = self.device_combo.current()
        if 0 <= idx < len(self.available_devices):
            d = self.available_devices[idx]
            self.info_text.delete('1.0','end')
            self.info_text.insert('end', f"Device: {d.name}\nVendor: {d.vendor}\nDLL: {d.function_lib}\n")

    # ---------- connect ----------
    def _connect(self):
        if self.j2534: return
        try:
            dll = self.dll_path.get().strip()
            if not dll: raise RuntimeError("Select a J2534 device.")
            bitrate = int(self.bitrate.get())
            self.j2534 = J2534(dll)
            self.j2534.open()
            self.j2534.connect_can(bitrate, use_iso15765=self.chk_iso.get(), extended_ids=self.chk_ext.get())
            self.status_label.config(text="● Connected", foreground="green")
            self.connect_btn.config(state='disabled'); self.disconnect_btn.config(state='normal')
            self._start_rx()
            self._log_monitor(f"[CONNECTED] {dll} @ {bitrate} bps")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            if self.j2534:
                try: self.j2534.close()
                except: pass
            self.j2534 = None

    def _disconnect(self):
        self._stop_rx(); self._stop_all_gen()
        if self.j2534:
            try: self.j2534.close()
            except: pass
            self.j2534=None
        self.status_label.config(text="● Disconnected", foreground="red")
        self.connect_btn.config(state='normal'); self.disconnect_btn.config(state='disabled')
        self._log_monitor("[DISCONNECTED]")

    # ---------- RX thread ----------
    def _start_rx(self):
        if self.rx_thread and self.rx_thread.is_alive(): return
        self.rx_running.set()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _stop_rx(self):
        self.rx_running.clear()
        if self.rx_thread:
            self.rx_thread.join(timeout=0.5)
            self.rx_thread=None

    def _rx_loop(self):
        last = time.time(); count=0
        while self.rx_running.is_set():
            try:
                if not self.j2534:
                    time.sleep(0.05); continue
                frames = self.j2534.read_can_batch(64, 50)
                for f in frames:
                    try: self.rx_queue.put_nowait(f)
                    except queue.Full: pass
                    self.stats['rx_total'] += 1; count += 1
                now = time.time()
                if now - last >= 1.0:
                    self.stats['current_rate']=count; count=0; last=now
            except Exception:
                self.stats['rx_errors'] += 1
                time.sleep(0.02)

    # ---------- GUI pump ----------
    def _start_gui_pump(self): self.after(25, self._gui_pump)

    def _gui_pump(self):
        mode = self.current_mode.get()
        batch=[]
        try:
            for _ in range(200):
                batch.append(self.rx_queue.get_nowait())
        except queue.Empty:
            pass

        for can_id, ext, data, ts in batch:
            # candump log file sink first (if active)
            if self.log_file:
                self.log_file.write(f"({ts:10d}) {('can0 ' if True else '')}{(f'{can_id:08X}' if ext else f'{can_id:03X}')}#{''.join(f'{b:02X}' for b in data)}\n")

            if mode == "candump" and not self.monitor_pause.get():
                if filter_match(can_id, self.dump_filter):
                    self._log_monitor(fmt_frame(can_id, ext, data, ts))
            elif mode == "cansniffer":
                if filter_match(can_id, self.dump_filter):
                    last = self.sniffer_last.get(can_id)
                    if last is None or last != data:
                        self.sniffer_last[can_id]=data
                        self._log_sniffer(fmt_frame(can_id, ext, data, ts))

        # stats line
        self._statlbl['rx_total'].config(text=str(self.stats['rx_total']))
        self._statlbl['tx_total'].config(text=str(self.stats['tx_total']))
        self._statlbl['rx_errors'].config(text=str(self.stats['rx_errors']))
        self._statlbl['current_rate'].config(text=str(self.stats['current_rate']))
        self.stats_label.config(text=f"RX:{self.stats['rx_total']} | TX:{self.stats['tx_total']} | Err:{self.stats['rx_errors']} | Rate:{self.stats['current_rate']} fps")
        self.after(25, self._gui_pump)

    # ---------- monitor ----------
    def _start_monitor(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        filt_str = self.monitor_filter.get().strip()
        self.dump_filter = parse_id_mask(filt_str) if filt_str else None
        # Try HW filter if present
        if self.dump_filter:
            base, mask = self.dump_filter
            try:
                self.j2534.start_pass_filter(base, mask, extended=self.chk_ext.get())
                self._log_monitor(f"[MONITOR] Hardware filter set: {filt_str}")
            except Exception as e:
                self._log_monitor(f"[MONITOR] HW filter not available: {e}. Using software filter.")
        self.current_mode.set("candump")
        self._log_monitor(f"[MONITOR STARTED] Filter: {filt_str or 'None'}")

    def _stop_monitor(self):
        self.current_mode.set("idle")
        self._log_monitor("[MONITOR STOPPED]")

    def _clear_text(self, widget):
        widget.delete('1.0','end')

    def _log_monitor(self, s:str):
        self.monitor_text.insert('end', s+"\n"); self.monitor_text.see('end')
        # trim
        if int(self.monitor_text.index('end-1c').split('.')[0]) > 12000:
            self.monitor_text.delete('1.0','1000.0')

    def _choose_log(self):
        if self.log_file:
            try: self.log_file.close()
            except: pass
            self.log_file = None
            messagebox.showinfo("Logging","Stopped logging.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".log", filetypes=[("candump log","*.log"),("All files","*.*")])
        if path:
            self.log_file = open(path, "w", buffering=1)
            messagebox.showinfo("Logging", f"Logging to {path}")

    # ---------- send ----------
    def _send_quick(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        try:
            can_id, ext, data = parse_cansend(self.send_input.get().strip())
            self.j2534.write_can(can_id, data, ext); self.stats['tx_total'] += 1
            self.tx_text.insert('end', f"[TX] {fmt_frame(can_id, ext, data)}\n"); self.tx_text.see('end')
            self.send_input.set("")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def _send_built(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        try:
            can_id = int(self.builder_id.get(),16); ext=self.builder_ext.get()
            data_str=self.builder_data.get().strip()
            if data_str and (not HEXD.match(data_str) or len(data_str)%2): raise ValueError("Data must be even-length hex")
            data = bytes.fromhex(data_str) if data_str else b''
            if len(data)>8: raise ValueError("Max 8 bytes")
            self.j2534.write_can(can_id, data, ext); self.stats['tx_total'] += 1
            self.tx_text.insert('end', f"[TX] {fmt_frame(can_id, ext, data)}\n"); self.tx_text.see('end')
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    # ---------- generator ----------
    def _start_gen(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        try:
            id_str = self.gen_id.get().strip()
            interval = max(0.0, float(self.gen_interval.get())/1000.0)
            dlc = int(self.gen_dlc.get()); random_data=self.gen_random_data.get(); random_id=self.gen_random_id.get()
            if ':' in id_str:
                a,b = id_str.split(':'); id_range=(int(a,16), int(b,16)); ids=None
            else:
                ids=[int(id_str,16)]; id_range=None
            stop=threading.Event()
            def worker():
                try:
                    while not stop.is_set():
                        if id_range: can_id=random.randint(id_range[0], id_range[1])
                        elif random_id and ids: can_id=random.choice(ids)
                        else: can_id=ids[0] if ids else 0x123
                        data = bytes(random.getrandbits(8) for _ in range(dlc)) if random_data else bytes([0x55]*dlc)
                        self.j2534.write_can(can_id, data, (can_id>0x7FF)); self.stats['tx_total']+=1
                        time.sleep(interval)
                except Exception:
                    pass
            t=threading.Thread(target=worker, daemon=True); t.start()
            self.generators.append((t,stop))
            self.gen_list.insert('end', f"Gen#{len(self.generators)}: {id_str}, {interval*1000:.0f}ms, DLC={dlc}\n"); self.gen_list.see('end')
        except Exception as e:
            messagebox.showerror("Generator Error", str(e))

    def _stop_all_gen(self):
        for t,stop in self.generators:
            stop.set()
        for t,_ in self.generators:
            t.join(timeout=0.5)
        self.generators.clear()
        self.gen_list.insert('end',"All generators stopped.\n"); self.gen_list.see('end')

    # ---------- sniffer ----------
    def _start_sniffer(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        filt_str = self.sniffer_filter.get().strip()
        self.dump_filter = parse_id_mask(filt_str) if filt_str else None
        self.sniffer_last.clear()
        self.current_mode.set("cansniffer")
        self._log_sniffer(f"[SNIFFER] Filter: {filt_str or 'None'} (prints only on data change)")

    def _stop_sniffer(self):
        self.current_mode.set("idle"); self._log_sniffer("[SNIFFER STOPPED]")

    def _log_sniffer(self, s:str):
        self.sniffer_text.insert('end', s+"\n"); self.sniffer_text.see('end')
        if int(self.sniffer_text.index('end-1c').split('.')[0])>12000:
            self.sniffer_text.delete('1.0','1000.0')

    # ---------- player (minimal hook) ----------
    def _load_log_for_playback(self):
        path = filedialog.askopenfilename(title="Open candump log", filetypes=[("Log","*.log *.txt *.asc"),("All","*.*")])
        if not path: return
        self.playback_frames=[]
        with open(path,'r') as f:
            for line in f:
                line=line.strip()
                if not line or line.startswith('#'): continue
                ts=None; s=line
                if line.startswith('(') and ')' in line:
                    i=line.find(')'); ts_str=line[1:i].strip()
                    try: ts=float(ts_str)
                    except: ts=None
                    s=line[i+1:].strip()
                parts=s.split()
                if len(parts)>=2 and (parts[0].startswith('can') or parts[0].startswith('vcan')):
                    s=parts[1]
                if '#' not in s: continue
                id_str,data_str=s.split('#',1)
                ext=len(id_str)>3; can_id=int(id_str,16)
                if data_str.upper().startswith('R'): continue
                data=bytes.fromhex(data_str) if data_str else b''
                self.playback_frames.append((can_id,ext,data,ts))
        self.player_status.set(f"Loaded {len(self.playback_frames)} frames from {os.path.basename(path)}")

    def _start_playback(self):
        if not self.j2534: return messagebox.showerror("Error","Not connected")
        if not self.playback_frames: return
        if self.playback_thread and self.playback_thread.is_alive(): return
        self.playback_running.set(); self.playback_paused.clear()
        def worker():
            try:
                sp=self.player_speed.get()
                # normalize to deltas
                deltas=[]
                t0=None
                for _,_,_,ts in self.playback_frames:
                    if ts is None: deltas=None; break
                    if t0 is None: t0=ts
                    deltas.append(ts - t0)
                for i,(cid,ext,data,_ts) in enumerate(self.playback_frames):
                    if not self.playback_running.is_set(): break
                    self.j2534.write_can(cid, data, ext); self.stats['tx_total']+=1
                    if deltas and i < len(deltas)-1:
                        dt = (deltas[i+1]-deltas[i]) / max(sp,0.01)
                        if dt>0: time.sleep(dt)
                    else:
                        time.sleep(0.001/max(sp,0.01))
            except Exception:
                pass
        self.playback_thread=threading.Thread(target=worker,daemon=True); self.playback_thread.start()
        self.player_status.set("Playback running…")

    def _stop_playback(self):
        self.playback_running.clear()
        if self.playback_thread:
            self.playback_thread.join(timeout=0.5)
            self.playback_thread=None
        self.player_status.set("Playback stopped.")

    # ---------- helpers ----------
    def _clear_and_focus(self, e=None): pass

# ===========================
# Main
# ===========================
def main():
    import struct, platform
    # Do the arch warning using a temporary hidden root so messagebox is safe pre-root
    arch_bits = struct.calcsize("P")*8
    if arch_bits != 32:
        tmp_root = tk.Tk(); tmp_root.withdraw()
        cont = messagebox.askyesno(
            "Architecture Warning",
            f"You are running {platform.architecture()[0]} Python.\n"
            "32-bit Python is required to load 32-bit J2534 DLLs.\n\nContinue anyway?",
            parent=tmp_root
        )
        tmp_root.destroy()
        if not cont:
            sys.exit(1)

    app = CANUtilitySuite()
    def on_close():
        if app.j2534:
            if not messagebox.askokcancel("Quit","Disconnect and exit?", parent=app): return
            app._disconnect()
        try:
            if app.log_file: app.log_file.close()
        except: pass
        app.destroy()
    app.protocol("WM_DELETE_WINDOW", on_close)
    app.mainloop()

if __name__ == "__main__":
    main()
