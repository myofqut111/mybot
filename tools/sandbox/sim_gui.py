#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mybot_sim 沙盒窗口。

窗口只是遥控器：所有配置都是 ~/mybot_sim 下的普通文件，
外部改了这里 2 秒后会自己重载；核心逻辑全在 simcore.py，
这个文件即使删掉也不影响命令行使用。
"""
import json
import os
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simcore                                     # noqa: E402

BG = "#171a1f"
PANEL = "#1e232a"
LINE = "#2b3138"
INK = "#e6edf3"
DIM = "#8b95a1"
GRIP = "#4b8bff"
GREEN = "#2ea043"
RED = "#d1453b"
GREY_BTN = "#2b3138"

FONT = ("Microsoft YaHei UI", 10)
FONT_S = ("Microsoft YaHei UI", 9)
FONT_T = ("Microsoft YaHei UI", 14, "bold")
MONO = ("DejaVu Sans Mono", 9)

SOURCES = ("全部", "探索", "导航", "注入", "其它")


class SandboxApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("mybot_sim · 沙盒")
        self.geometry("1360x860+40+40")
        self.minsize(1180, 760)
        self.configure(bg=BG)

        self.state_data = simcore.load_state()
        self.fault_vars = {}          # name -> (BooleanVar, {arg: StringVar})
        self.param_rows = []          # (file, path, kind, var)
        self.param_mtime = {}
        self.log_path = None
        self.log_pos = 0
        self.log_filter = tk.StringVar(value="全部")
        self.log_search = tk.StringVar()
        self.running = False

        self._build_top()
        self._build_status()      # 先占住底部，避免被 body 挤出去
        self._build_body()

        self.reload_scenarios()
        self.reload_params()
        self.reload_faults()
        self.after(300, self._poll)
        self.after(2000, self._watch_files)

    # ------------------------------------------------------------ 顶栏
    def _build_top(self):
        bar = tk.Frame(self, bg=BG)
        bar.pack(side="top", fill="x", padx=14, pady=10)

        tk.Label(bar, text="mybot_sim · 沙盒", bg=BG, fg=INK,
                 font=FONT_T).pack(side="left")

        self.pill_domain = self._pill(bar, "domain %s" % self.state_data.get("domain", 42))
        self.pill_port = self._pill(bar, "端口 %s" % self.state_data.get("http_port", 8180))
        self.pill_state = self._pill(bar, "未运行")

        for text, cmd, color in (
                ("重置", self.on_reset, GREY_BTN),
                ("停止", self.on_stop, RED),
                ("启动", self.on_start, GREEN),
                ("RViz", self.on_rviz, GREY_BTN),
                ("源码", self.on_open_src, GREY_BTN),
                ("终端", self.on_open_terminal, GREY_BTN)):
            tk.Button(bar, text=text, command=cmd, bg=color, fg="white",
                      relief="flat", font=FONT, padx=16, pady=6,
                      activebackground=color, cursor="hand2").pack(side="right", padx=4)

    def _pill(self, parent, text):
        lbl = tk.Label(parent, text=text, bg=GREY_BTN, fg=DIM, font=FONT_S,
                       padx=12, pady=5)
        lbl.pack(side="left", padx=6)
        return lbl

    # ------------------------------------------------------------ 主体
    def _build_body(self):
        body = tk.Frame(self, bg=BG)
        body.pack(side="top", fill="both", expand=True, padx=14, pady=(0, 8))
        body.columnconfigure(0, weight=0, minsize=520)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(3, weight=1)
        left.columnconfigure(0, weight=1)
        self._build_scenarios(left)
        self._build_faults(left)
        self._build_params(left)
        self._build_log(body)

    def _card(self, parent, title, hint="", row=None):
        outer = tk.Frame(parent, bg=PANEL, highlightbackground=LINE,
                         highlightthickness=1)
        if row is None:
            outer.pack(side="top", fill="x", pady=(0, 10))
        else:
            outer.grid(row=row, column=0, sticky="nsew", pady=(0, 10))
        head = tk.Frame(outer, bg=PANEL)
        head.pack(side="top", fill="x", padx=12, pady=(10, 4))
        tk.Label(head, text=title, bg=PANEL, fg=INK, font=("Microsoft YaHei UI", 11, "bold")).pack(side="left")
        if hint:
            tk.Label(head, text=hint, bg=PANEL, fg=DIM, font=FONT_S).pack(side="right")
        return outer

    # 场景
    def _build_scenarios(self, parent):
        card = self._card(parent, "场景", "扫 scenarios/", row=1)
        self.scn_list = tk.Listbox(card, bg=PANEL, fg=INK, font=FONT_S,
                                   selectbackground=GRIP, selectforeground="white",
                                   highlightthickness=0, borderwidth=0, height=4)
        self.scn_list.pack(side="top", fill="x", padx=12)

        cmd_row = tk.Frame(card, bg=PANEL)
        cmd_row.pack(side="top", fill="x", padx=12, pady=(8, 12))
        tk.Label(cmd_row, text="自由命令", bg=PANEL, fg=DIM, font=FONT_S).pack(side="left")
        self.cmd_var = tk.StringVar(value="ros2 node list")
        entry = tk.Entry(cmd_row, textvariable=self.cmd_var, bg="#141821", fg=INK,
                         insertbackground=INK, relief="flat", font=MONO)
        entry.pack(side="left", fill="x", expand=True, padx=8, ipady=3)
        tk.Button(cmd_row, text="执行", command=self.on_command, bg=GRIP, fg="white",
                  relief="flat", font=FONT_S, padx=14, cursor="hand2").pack(side="right")

    # 参数
    def _build_params(self, parent):
        card = tk.Frame(parent, bg=PANEL, highlightbackground=LINE, highlightthickness=1)
        card.grid(row=3, column=0, sticky="nsew", pady=(0, 10))
        head = tk.Frame(card, bg=PANEL)
        head.pack(side="top", fill="x", padx=12, pady=(10, 4))
        tk.Label(head, text="参数", bg=PANEL, fg=INK,
                 font=("Microsoft YaHei UI", 11, "bold")).pack(side="left")
        tk.Label(head, text="扫 config/*.yaml（标量）", bg=PANEL, fg=DIM,
                 font=FONT_S).pack(side="right")
        wrap = tk.Frame(card, bg=PANEL)
        wrap.pack(side="top", fill="both", expand=True, padx=12, pady=(0, 4))
        self.param_canvas = tk.Canvas(wrap, bg=PANEL, highlightthickness=0, height=200)
        scroll = tk.Scrollbar(wrap, orient="vertical", command=self.param_canvas.yview)
        self.param_canvas.configure(yscrollcommand=scroll.set)
        self.param_canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.param_inner = tk.Frame(self.param_canvas, bg=PANEL)
        self.param_canvas.create_window((0, 0), window=self.param_inner, anchor="nw")
        self.param_inner.bind(
            "<Configure>",
            lambda e: self.param_canvas.configure(scrollregion=self.param_canvas.bbox("all")))

        row = tk.Frame(card, bg=PANEL)
        row.pack(side="bottom", fill="x", padx=12, pady=(6, 12))
        tk.Button(row, text="应用", command=self.on_apply_params, bg=GRIP, fg="white",
                  relief="flat", font=FONT_S, padx=14, cursor="hand2").pack(side="right")
        self.param_hint = tk.Label(row, text="改完点应用，重启场景后生效", bg=PANEL,
                                   fg=DIM, font=FONT_S)
        self.param_hint.pack(side="left")

    # 故障
    def _build_faults(self, parent):
        card = self._card(parent, "故障注入", "扫 faults/", row=2)
        self.fault_box = tk.Frame(card, bg=PANEL)
        self.fault_box.pack(side="top", fill="x", padx=12, pady=(0, 8))
        tk.Label(card, text="faults/ 里的脚本会被动态执行，别放来路不明的代码",
                 bg=PANEL, fg="#c9635a", font=FONT_S, anchor="w").pack(
                     side="top", fill="x", padx=12, pady=(0, 10))

    # 日志
    def _build_log(self, parent):
        card = tk.Frame(parent, bg=PANEL, highlightbackground=LINE, highlightthickness=1)
        card.grid(row=0, column=1, sticky="nsew")
        head = tk.Frame(card, bg=PANEL)
        head.pack(side="top", fill="x", padx=12, pady=(10, 6))
        tk.Label(head, text="日志", bg=PANEL, fg=INK,
                 font=("Microsoft YaHei UI", 11, "bold")).pack(side="left")

        for name in SOURCES[1:]:
            tk.Radiobutton(head, text=name, value=name, variable=self.log_filter,
                           command=self._redraw_log, bg=PANEL, fg=DIM,
                           selectcolor=PANEL, activebackground=PANEL,
                           font=FONT_S, borderwidth=0,
                           highlightthickness=0).pack(side="left", padx=2)

        search = tk.Entry(head, textvariable=self.log_search, bg="#141821", fg=INK,
                          insertbackground=INK, relief="flat", width=18, font=FONT_S)
        search.pack(side="right", ipady=3)
        search.bind("<KeyRelease>", lambda e: self._redraw_log())
        tk.Label(head, text="关键字", bg=PANEL, fg=DIM, font=FONT_S).pack(side="right", padx=6)

        self.log_text = tk.Text(card, bg="#0f1216", fg="#cfd8e3", font=MONO,
                                relief="flat", wrap="none", insertbackground=INK)
        self.log_text.pack(side="left", fill="both", expand=True, padx=(12, 0), pady=(0, 12))
        sb = tk.Scrollbar(card, orient="vertical", command=self.log_text.yview)
        sb.pack(side="right", fill="y", pady=(0, 12), padx=(0, 4))
        self.log_text.configure(yscrollcommand=sb.set)
        self.log_text.tag_configure("err", foreground="#ff7b72")
        self.log_text.tag_configure("warn", foreground="#e3b341")
        self.log_text.tag_configure("ok", foreground="#7ee787")
        self.log_text.tag_configure("dim", foreground="#5c6672")
        self.log_text.insert("end", "还没有日志：点「启动」跑一个场景，这里会实时滚动。\n", "dim")
        self._placeholder = True

    def _build_status(self):
        bar = tk.Frame(self, bg=BG)
        bar.pack(side="bottom", fill="x", padx=14, pady=(0, 10))
        self.status_left = tk.Label(bar, text="", bg=BG, fg=DIM, font=FONT_S, anchor="w")
        self.status_left.pack(side="left")
        self.status_right = tk.Label(
            bar, text="domain %s · cwd ~/mybot_sim" % self.state_data.get("domain", 42),
            bg=BG, fg=DIM, font=FONT_S, anchor="e")
        self.status_right.pack(side="right")

    # ------------------------------------------------------------ 数据装载
    def reload_scenarios(self):
        self.scn_list.delete(0, "end")
        self.scenarios = simcore.list_scenarios()
        for item in self.scenarios:
            self.scn_list.insert("end", "  " + item["name"])
        if self.scenarios:
            self.scn_list.selection_set(0)

    def reload_params(self):
        for child in self.param_inner.winfo_children():
            child.destroy()
        self.param_rows = []
        current_file = None
        for row in simcore.list_params():
            if row["file"] != current_file:
                current_file = row["file"]
                tk.Label(self.param_inner, text=row["file"], bg=PANEL, fg=DIM,
                         font=("Microsoft YaHei UI", 9, "bold"), anchor="w").pack(
                             side="top", fill="x", pady=(6, 2))
            line = tk.Frame(self.param_inner, bg=PANEL)
            line.pack(side="top", fill="x")
            tk.Label(line, text="   " + str(row["key"]), bg=PANEL, fg=INK,
                     font=FONT_S, width=22, anchor="w").pack(side="left")
            if row["kind"] == "complex":
                tk.Label(line, text=str(row["value"]), bg=PANEL, fg="#c9635a",
                         font=FONT_S, anchor="w").pack(side="left")
                continue
            var = tk.StringVar(value=str(row["value"]))
            entry = tk.Entry(line, textvariable=var, bg="#141821", fg=INK,
                             insertbackground=INK, relief="flat", font=MONO, width=20)
            entry.pack(side="left", ipady=2)
            self.param_rows.append((row["file"], row["path"], row["kind"], var))
            self.param_mtime[row["file"]] = os.path.getmtime(
                os.path.join(simcore.ROOT, "config", row["file"]))
        if not self.param_rows:
            tk.Label(self.param_inner, text="config/ 里还没有 yaml",
                     bg=PANEL, fg=DIM, font=FONT_S).pack(side="top", anchor="w", pady=6)

    def reload_faults(self):
        for child in self.fault_box.winfo_children():
            child.destroy()
        self.fault_vars = {}
        for item in simcore.list_faults():
            line = tk.Frame(self.fault_box, bg=PANEL)
            line.pack(side="top", fill="x", pady=1)
            var = tk.BooleanVar(value=False)
            tk.Checkbutton(line, variable=var, bg=PANEL, fg=INK, selectcolor=PANEL,
                           activebackground=PANEL, borderwidth=0,
                           highlightthickness=0,
                           command=self.on_fault_toggle).pack(side="left")
            label = item["label"] if item["label"] != item["name"] else item["name"]
            tk.Label(line, text=label, bg=PANEL, fg=INK, font=FONT_S,
                     width=12, anchor="w").pack(side="left")
            if item["error"]:
                tk.Label(line, text="载入失败：" + item["error"], bg=PANEL,
                         fg="#c9635a", font=FONT_S, anchor="w").pack(side="left")
            args = {}
            for arg, default in (item["args"] or {}).items():
                tk.Label(line, text=arg, bg=PANEL, fg=DIM, font=FONT_S).pack(side="left", padx=(6, 2))
                arg_var = tk.StringVar(value=str(default))
                entry = tk.Entry(line, textvariable=arg_var, bg="#141821", fg=INK,
                                 insertbackground=INK, relief="flat", font=MONO,
                                 width=8)
                entry.pack(side="left", ipady=1)
                entry.bind("<Return>", lambda e: self.on_fault_toggle())
                entry.bind("<FocusOut>", lambda e: self.on_fault_toggle())
                args[arg] = arg_var
            tk.Label(line, text=item["topic"] or "", bg=PANEL, fg=DIM,
                     font=FONT_S).pack(side="right", padx=6)
            self.fault_vars[item["name"]] = (var, args)

    # ------------------------------------------------------------ 动作
    def on_start(self):
        if self.running:
            messagebox.showinfo("沙盒", "已经在跑了，先停止")
            return
        sel = self.scn_list.curselection()
        if not sel:
            messagebox.showwarning("沙盒", "先选一个场景")
            return
        name = self.scenarios[sel[0]]["name"]
        try:
            simcore.do_run(type("A", (), {"scenario": name})())
        except SystemExit as error:
            messagebox.showerror("沙盒", str(error))
            return
        self.state_data = simcore.load_state()
        self.log_path = self._latest_log()
        self.log_pos = 0
        self.log_text.delete("1.0", "end")
        self.running = True
        self.pill_state.configure(text="运行中", fg="white", bg=GREEN)
        self.status_left.configure(text="运行中 · %s" % name)
        self.write_faults_snapshot()

    def on_stop(self):
        simcore.do_stop(type("A", (), {})())
        self.running = False
        self.pill_state.configure(text="未运行", fg=DIM, bg=GREY_BTN)
        self.status_left.configure(text="已停止")

    def on_reset(self):
        if not messagebox.askyesno("重置", "清空编译产物与 data/，源码和场景保留。继续？"):
            return
        simcore.do_reset(type("A", (), {})())
        messagebox.showinfo("重置", "已清空")

    def on_rviz(self):
        env = simcore.child_env(self.state_data.get("domain", 42),
                                self.state_data.get("http_port", 8180))
        script = ("source /opt/ros/humble/setup.bash; "
                  "source $HOME/fishbot_full_ws/install/setup.bash; rviz2")
        try:
            subprocess.Popen(["x-terminal-emulator", "-e", "bash", "-lc", script], env=env)
        except FileNotFoundError:
            messagebox.showwarning("RViz", "没找到终端程序，自己开一个终端跑 rviz2 吧")

    def on_command(self):
        text = self.cmd_var.get().strip()
        if not text:
            return
        env = simcore.child_env(self.state_data.get("domain", 42),
                                self.state_data.get("http_port", 8180))
        script = ("source /opt/ros/humble/setup.bash; "
                  "source $HOME/fishbot_full_ws/install/setup.bash; " + text)
        try:
            out = subprocess.run(["bash", "-lc", script], env=env, cwd=simcore.ROOT,
                                 capture_output=True, text=True, timeout=20)
            body = (out.stdout or "") + (out.stderr or "")
        except subprocess.TimeoutExpired:
            body = "（超时 20 秒，命令可能还在跑）"
        self._append_line("17:%s [command] %s\n%s" % (
            time.strftime("%H:%M"), text, body.strip()[:4000]))

    def on_apply_params(self):
        changed = []
        for filename, path, kind, var in self.param_rows:
            try:
                old, new = simcore.set_param(filename, path, var.get())
                if str(old) != str(new):
                    changed.append("%s %s: %s → %s" % (filename, path, old, new))
            except Exception as error:            # noqa: BLE001
                messagebox.showerror("参数", "写 %s 失败：%s" % (path, error))
                return
        self.write_faults_snapshot()
        self.param_hint.configure(
            text=("已应用 %d 项，重启场景后生效" % len(changed)) if changed else "没有改动")
        for line in changed:
            self._append_line("[params] " + line)

    def on_fault_toggle(self):
        """勾选/改参数即时写 faults.json —— 故障桥 1 秒内自己重载，不用重启。"""
        active = self.write_faults_snapshot()
        if active:
            self._append_line("[fault] 已启用：%s" % json.dumps(active, ensure_ascii=False))
        else:
            self._append_line("[fault] 已全部关闭")

    def on_open_terminal(self):
        env = simcore.child_env(self.state_data.get("domain", 42),
                                self.state_data.get("http_port", 8180))
        script = ("cd %s; source /opt/ros/humble/setup.bash; "
                  "source $HOME/fishbot_full_ws/install/setup.bash; "
                  "echo '沙盒终端：ROS_DOMAIN_ID='$ROS_DOMAIN_ID; exec bash" % simcore.ROOT)
        for term in ("x-terminal-emulator", "gnome-terminal", "xterm"):
            try:
                if term == "gnome-terminal":
                    subprocess.Popen([term, "--", "bash", "-lc", script], env=env)
                else:
                    subprocess.Popen([term, "-e", "bash", "-lc", script], env=env)
                return
            except FileNotFoundError:
                continue
        messagebox.showwarning("终端", "没找到终端程序，手动开一个吧")

    def on_open_src(self):
        path = os.path.join(simcore.WS, "src")
        for cmd in (["xdg-open", path], ["nautilus", path], ["thunar", path]):
            try:
                subprocess.Popen(cmd)
                return
            except FileNotFoundError:
                continue
        messagebox.showinfo("源码", "源码在：%s" % path)

    def write_faults_snapshot(self):
        active = {}
        for name, (var, args) in self.fault_vars.items():
            if not var.get():
                continue
            active[name] = {k: v.get() for k, v in args.items()}
        path = os.path.join(simcore.ROOT, "faults.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(active, fh, ensure_ascii=False, indent=2)
        return active

    # ------------------------------------------------------------ 轮询
    def _latest_log(self):
        base = os.path.join(simcore.ROOT, "logs")
        if not os.path.isdir(base):
            return None
        runs = sorted(d for d in os.listdir(base) if
                      os.path.isdir(os.path.join(base, d)))
        if not runs:
            return None
        return os.path.join(base, runs[-1], "console.log")

    def _poll(self):
        if self.log_path is None:
            self.log_path = self._latest_log()
        if self.log_path and os.path.exists(self.log_path):
            try:
                with open(self.log_path, encoding="utf-8", errors="replace") as fh:
                    fh.seek(self.log_pos)
                    chunk = fh.read()
                    self.log_pos = fh.tell()
            except OSError:
                chunk = ""
            if chunk:
                self._append_line(chunk)
        pid = simcore.read_pid()
        if self.running and not simcore.alive(pid):
            self.running = False
            self.pill_state.configure(text="已退出", fg="white", bg=RED)
            self.status_left.configure(text="场景已退出（看日志尾部）")
        self.after(400, self._poll)

    def _watch_files(self):
        """外部改了 yaml 就重载，避免两头改打架。"""
        if not self.running:
            try:
                changed = []
                for name in os.listdir(os.path.join(simcore.ROOT, "config")):
                    if not name.endswith((".yaml", ".yml")):
                        continue
                    path = os.path.join(simcore.ROOT, "config", name)
                    mtime = os.path.getmtime(path)
                    if self.param_mtime.get(name) not in (None, mtime):
                        changed.append(name)
                    self.param_mtime[name] = mtime
                if changed:
                    self.reload_params()
                    self._append_line(
                        "[params] 检测到外部修改，已重新加载：%s\n" % ", ".join(changed))
            except OSError:
                pass
        self.after(2000, self._watch_files)

    # ------------------------------------------------------------ 日志渲染
    def _append_line(self, text):
        if getattr(self, "_placeholder", False):
            self.log_text.delete("1.0", "end")
            self._placeholder = False
        for line in text.splitlines():
            tag = ""
            low = line.lower()
            if "error" in low or "traceback" in low or "异常" in line:
                tag = "err"
            elif "warn" in low:
                tag = "warn"
            elif "收工" in line or "已保存" in line or "ready" in low:
                tag = "ok"
            self.log_text.insert("end", line + "\n", tag)
        self.log_text.see("end")

    def _redraw_log(self):
        """按来源/关键字重画（只保留最近 3000 行）。"""
        if not self.log_path or not os.path.exists(self.log_path):
            return
        try:
            with open(self.log_path, encoding="utf-8", errors="replace") as fh:
                lines = fh.readlines()[-3000:]
        except OSError:
            return
        want = self.log_filter.get()
        key = self.log_search.get().strip()
        self.log_text.delete("1.0", "end")
        for line in lines:
            if key and key not in line:
                continue
            src = self._source_of(line)
            if want != "全部" and src != want:
                continue
            self._append_line(line)


    @staticmethod
    def _source_of(line):
        if "[fault]" in line or "注入" in line:
            return "注入"
        if "explore" in line:
            return "探索"
        if any(k in line for k in ("nav2", "amcl", "planner", "controller", "lifecycle")):
            return "导航"
        return "其它"


def main():
    app = SandboxApp()
    app.mainloop()


if __name__ == "__main__":
    main()
