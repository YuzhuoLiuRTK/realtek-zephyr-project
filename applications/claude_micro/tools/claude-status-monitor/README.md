# Claude Code Teams Notifier — Python 版

<div align="center">

**在 Claude Code 需要确认 / 任务完成时,自动给 Teams 发消息(纯 Python,非阻塞)**

[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS%20%7C%20Linux-blue.svg)](#)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](#)

</div>

---

## ✨ 功能特性

🔔 **两个关键节点通知** —— 通过 Claude Code hooks 自动捕获,无需手动干预:
- 🟠 **等待操作** —— 弹出权限确认时(`PermissionRequest`),提醒你切回终端处理。
- ✅ **任务完成** —— 主循环结束时(`Stop`),告诉你这一轮已处理完。

⚡ **非阻塞** —— hook 进程只负责派生一个脱离的后台 worker 然后立刻退出,真正的网络 POST(最长 10s 超时)在后台完成,**绝不卡住 Claude Code**。

🏷️ **带项目名** —— 消息标题里带上当前项目目录名(取自 `CLAUDE_PROJECT_DIR`),多个会话同时跑也能分清是哪个。

📦 **轻量** —— Teams 通知仅依赖 `requests`(发 HTTP),其余全是标准库。

🖥️ **状态面板 / 设备联动(可选)** —— 桌面面板 + claude_micro USB 设备,见下文「状态面板 + claude_micro 设备」一节。

---

## 📦 运行要求

- Python **3.8+**
- `pip install requests`
- 一个 Teams **Power Automate / Workflow** 的 Webhook URL(用于接收 Adaptive Card)

---

## 🚀 快速开始

### 1. 配置 Teams Workflow

通知通过 Teams 的 **Workflows(Power Automate)** 插件接收,下面一步步拿到 Webhook URL。

**(1) 安装 Workflows 插件**

在 Teams 客户端的应用市场里搜索并安装 **Workflows** 插件。

![安装 Workflows 插件](doc/images/insatll_workflows.png)

**(2) 创建一个团队(Team)用于接收消息**

新建或选择一个团队 / 频道,通知消息会发到这里。

![创建团队](doc/images/create_a_team.png)

**(3) 新建 Workflow**

进入 Workflows,点击新建一个工作流。

![新建 Workflow](doc/images/creat_workflows.png)

**(4) 选择 webhook 模板并绑定频道**

选择 **send webhook alerts to a channel** 模板,把它绑定到上一步创建的频道。

![选择 webhook 模板](doc/images/send_webhook_alerts_to_a_channel.png)

![配置 Workflow](doc/images/config_workflows.png)

**(5) 复制 Webhook 链接**

工作流创建完成后,复制生成的 Webhook URL —— 这就是下一步要配置到环境变量 `TEAMS_WORKFLOW_URL` 的地址。

![复制 Webhook 链接](doc/images/copy_webhook_link.png)

**(6) 开启消息通知(可选)**

打开频道的桌面通知,这样收到消息时会有弹窗提醒。

![开启通知](doc/images/open_notification.png)

### 2. 配置 Workflow URL

`notify.py` 从环境变量 `TEAMS_WORKFLOW_URL` 读取 Webhook 地址(URL 带签名,不要写进代码仓库)。未设置时跳过 Teams 通知。

Windows(PowerShell,对当前用户永久生效,设置后需重启 Claude Code):

```powershell
[Environment]::SetEnvironmentVariable('TEAMS_WORKFLOW_URL', '<上一步复制的 Webhook URL>', 'User')
```

Linux / macOS:

```bash
export TEAMS_WORKFLOW_URL='<上一步复制的 Webhook URL>'
```

### 3. 配置 Claude Code hooks

编辑 Claude Code 配置文件:

- **Windows**:`%USERPROFILE%\.claude\settings.json`
- **macOS / Linux**:`~/.claude/settings.json`

把 `<项目路径>` 替换成你本地仓库的绝对路径,按需把 `python` 换成可用的解释器:

```json
{
  "hooks": {
    "PermissionRequest": [
      {
        "matcher": "",
        "hooks": [
          { "type": "command", "command": "python <项目路径>/notify.py waiting" }
        ]
      }
    ],
    "Stop": [
      {
        "matcher": "",
        "hooks": [
          { "type": "command", "command": "python <项目路径>/notify.py done" }
        ]
      }
    ]
  }
}
```

两个事件对应两条消息:

| Hook | 触发时机 | 事件 | 消息 |
|------|----------|------|------|
| `PermissionRequest` | 弹出权限授权请求时 | `waiting` | `[项目名] 等待操作` |
| `Stop` | 主循环结束(回到等待用户输入) | `done` | `[项目名] 任务完成` |

> **为什么用 `PermissionRequest` 而不是 `Notification`**:`Notification` 既会在「需要授权」时触发,也会在「闲置约 60 秒」时触发,会在你纯粹离开时误发。`PermissionRequest` 语义清晰,只在真正需要确认时触发。
>
> **路径写法**:Windows 上若用 Git Bash / WSL 风格路径(如 `/e/RealtekTool/...`),需确保被调用的 `python` 能解析它;否则用反斜杠绝对路径(`E:\\RealtekTool\\...`,JSON 里反斜杠要转义)。

### 4. 手动测试 / 调试

不依赖 Claude Code 也可以直接验证:

```bash
# 前台路径:派生后台 worker 后立刻退出(应当秒回)
python notify.py waiting
python notify.py done

# worker 路径:在当前进程里直接发(用于排查发送是否成功)
python notify.py --worker done "项目名"
```

---

## 🖥️ 状态面板 + claude_micro 设备(可选)

除 Teams 通知外,本目录还提供一个桌面状态面板,并可配合 `applications/claude_micro` 固件
(RTL87x2J EVB)使用:设备上的 3 个窗口灯显示最多 3 个 Claude Code 会话的状态,
3 个 YES 键用于在确认模式下放行对应会话的工具调用,麦克风键按住说话,语音经 Whisper
转写后粘贴进当前终端。

### 安装依赖

```bash
pip install -r requirements.txt
```

### 额外的 hooks

在上一节 `notify.py` 的基础上,再加入下面几条(同一事件可以挂多个 hook):

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "",
        "hooks": [
          { "type": "command", "command": "python <项目路径>/state_writer.py pre_tool" },
          { "type": "command", "command": "python <项目路径>/gate_hook.py" }
        ]
      }
    ],
    "PermissionRequest": [
      { "matcher": "", "hooks": [ { "type": "command", "command": "python <项目路径>/state_writer.py permission" } ] }
    ],
    "Stop": [
      { "matcher": "", "hooks": [ { "type": "command", "command": "python <项目路径>/state_writer.py stop" } ] }
    ]
  }
}
```

### 运行

```bash
python monitor_ui.py        # 状态面板;会自动在后台拉起 device_listener.py
python led_test_flow.py     # 不开 Claude Code,单独演示一个窗口灯的状态流转
```

所有进程通过 `~/.claude/monitor/` 下的文件交换状态:

| 文件 | 写入方 | 内容 |
|------|--------|------|
| `state_<key>.json` | `state_writer.py` | 会话状态 working / waiting / success |
| `gate_<key>.json` | `monitor_ui.py`、`gate_hook.py`、`device_listener.py` | 确认模式开关、待确认的工具、allow / block 决定 |
| `voice_result.json` | `device_listener.py` | 最近一次语音转写结果 |

`<key>` 是 `CLAUDE_PROJECT_DIR` 的 MD5 前 8 位,窗口按会话创建时间排序。

| 状态 | 面板 | 设备窗口灯 |
|------|------|-----------|
| idle | 灰 | 灭 |
| working | 蓝,慢闪 | 慢闪 |
| waiting | 黄,快闪 | 快闪 |
| success | 绿(10 s 后回到 idle) | 常亮 |

> **确认模式**:面板上每个窗口的确认开关默认打开。打开时 `gate_hook.py` 会挂起每一次工具调用,
> 等待面板上的 Yes / No 或设备上的 YES 键,60 s 无操作自动放行。

---

## ⚙️ 非阻塞原理

Claude Code 的 hook 是**同步**执行的 —— 它会等 hook 命令退出后才继续。如果在 hook 里直接 `requests.post(..., timeout=10)`,网络慢时最坏会把 hook 卡住 **10 秒**,确认框延迟弹出、收尾变慢。

`notify.py` 的做法:

```
hook 触发 → 前台进程 spawn 脱离的后台 worker → 前台立刻退出(~0.5s)
                                  └→ worker 独立完成 HTTP POST(可慢可失败,与 hook 无关)
```

- Windows 下用 `DETACHED_PROCESS | CREATE_NO_WINDOW` 派生,不挂在父进程上、不弹窗。
- 标准输入/输出全部重定向到 `DEVNULL`,worker 完全静默。
- worker 即使超时或失败,也只影响后台进程,**不影响 Claude Code 的关键路径**。

---

## 🗂️ 目录结构

```
.
├── notify.py                 # hook 入口:派生后台 worker 发 Teams,非阻塞
├── TeamsWorkflowNotifier.py  # 封装 Power Automate Adaptive Card 发送
├── state_writer.py           # hook 入口:写会话状态文件
├── gate_hook.py              # PreToolUse hook:确认模式下挂起工具调用,等待放行
├── monitor_ui.py             # Tk 状态面板(3 个窗口 + Yes/No + 确认开关)
├── device_listener.py        # claude_micro 设备:HID 按键/窗口灯 + UAC 麦克风 → Whisper
├── led_test_flow.py          # 窗口灯状态流转测试
├── Realtek.png               # 面板 logo
├── requirements.txt
├── doc/
│   ├── architecture.html     # 架构说明
│   └── images/               # README 配置截图
└── README.md                 # 本文件
```

---

## 🛠️ 技术栈

- **requests** —— 发起 HTTP POST(Adaptive Card)
- **subprocess(detached)** —— 把网络请求甩到后台,保证 hook 非阻塞
- **stdlib** —— `os` / `sys` / `subprocess` / `pathlib` / `datetime` / `json`

---

