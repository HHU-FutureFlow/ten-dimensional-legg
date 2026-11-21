using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Windows.Forms;

internal struct KeyMouse
{
    internal ushort Keys;
    internal short X, Y, Wheel;
    internal bool Left, Right;
}

internal static class Vt02Frame
{
    internal const int Size = 21;
    internal const ushort StopKey = 1 << 15;

    internal static void Encode(byte[] frame, byte sequence, KeyMouse input)
    {
        frame[0] = 0xA5;
        Write16(frame, 1, 12);
        frame[3] = sequence;
        frame[4] = (byte)Crc(frame, 4, 0xFF, 0x8C);
        Write16(frame, 5, 0x0304);
        Write16(frame, 7, input.X);
        Write16(frame, 9, input.Y);
        Write16(frame, 11, input.Wheel);
        frame[13] = input.Left ? (byte)1 : (byte)0;
        frame[14] = input.Right ? (byte)1 : (byte)0;
        Write16(frame, 15, input.Keys);
        Write16(frame, 17, 0);
        Write16(frame, 19, Crc(frame, 19, 0xFFFF, 0x8408));
    }

    private static void Write16(byte[] frame, int offset, int value)
    {
        frame[offset] = unchecked((byte)value);
        frame[offset + 1] = unchecked((byte)(value >> 8));
    }

    private static int Crc(byte[] frame, int length, int crc, int polynomial)
    {
        for (int i = 0; i < length; ++i)
        {
            crc ^= frame[i];
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 1) != 0 ? (crc >> 1) ^ polynomial : crc >> 1;
        }
        return crc;
    }
}

internal sealed class TeleopWindow : Form
{
    private const int FrameIntervalMs = 20;
    private const int SendStallMs = 100;
    private const int StopRepeatMs = 100;
    private const int WmInput = 0x00FF;
    private const int RightMouseButton = 0x02;
    private const uint RawInputRead = 0x10000003;
    private const ushort MouseMoveAbsolute = 0x0001;
    private const ushort MouseWheelFlag = 0x0400;
    private static readonly int[] ProtocolKeys = {
        'W', 'S', 'A', 'D', 0x10, 0x11, 'Q', 'E', 'R', 'F', 'G', 'Z', 'X', 'C', 'V', 'B'
    };
    private static readonly string[] ProtocolKeyNames = {
        "W", "S", "A", "D", "Shift", "Ctrl", "Q", "E", "R", "F", "G", "Z", "X", "C", "V", "B"
    };

    private readonly ComboBox ports = new ComboBox();
    private readonly Button refresh = new Button();
    private readonly Button connect = new Button();
    private readonly Label link = new Label();
    private readonly Label inputStatus = new Label();
    private readonly Label mouseStatus = new Label();
    private readonly Timer timer = new Timer();
    private readonly Stopwatch clock = Stopwatch.StartNew();
    private readonly byte[] frame = new byte[Vt02Frame.Size];
    private SerialPort serial;
    private long frameCount;
    private long nextFrameAt;
    private long lastWriteAt;
    private long repeatStopUntil;
    private byte sequence;
    private bool acceptInput;
    private bool rawMouseReady;
    private int mouseX, mouseY, wheel;

    internal TeleopWindow()
    {
        Text = "轮腿键鼠遥控";
        Font = new Font("Microsoft YaHei UI", 10F);
        ClientSize = new Size(510, 270);
        FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = false;
        StartPosition = FormStartPosition.CenterScreen;
        KeyPreview = true;
        AutoScaleMode = AutoScaleMode.Font;

        var layout = new FlowLayoutPanel {
            Dock = DockStyle.Fill, FlowDirection = FlowDirection.TopDown,
            WrapContents = false, Padding = new Padding(16), AutoScroll = true
        };
        var serialRow = new FlowLayoutPanel { AutoSize = true, WrapContents = false };
        serialRow.Controls.Add(new Label { Text = "串口", AutoSize = true, Margin = new Padding(0, 7, 8, 0) });
        ports.DropDownStyle = ComboBoxStyle.DropDownList;
        ports.Width = 104;
        ports.SelectedIndexChanged += delegate { connect.Enabled = serial != null || ports.SelectedItem != null; };
        serialRow.Controls.Add(ports);
        refresh.Text = "刷新";
        refresh.AutoSize = true;
        refresh.Click += delegate { RefreshPorts(); };
        serialRow.Controls.Add(refresh);
        connect.Text = "连接";
        connect.AutoSize = true;
        connect.Click += delegate { ToggleConnection(); };
        serialRow.Controls.Add(connect);

        layout.Controls.Add(serialRow);
        AddLine(layout, link, "未连接", 8);
        AddLine(layout, inputStatus, "输入  —", 3);
        AddLine(layout, mouseStatus, "鼠标  —", 3);
        AddLine(layout, new Label(), "R 允许键鼠控制      B 零力急停", 13);
        AddLine(layout, new Label(), "W/S 前后   A/D 或鼠标转向", 3);
        AddLine(layout, new Label(), "Z / X / C 腿长   0.15 / 0.25 / 0.35 m", 3);
        Controls.Add(layout);

        timer.Interval = 10;
        timer.Tick += delegate { SendCurrentInput(); };
        Shown += delegate { RefreshPorts(); };
        Deactivate += delegate { OnFocusLost(); };
        FormClosing += delegate { ClosePort("未连接", true); };
        FormClosed += delegate { timer.Dispose(); };
        KeyDown += delegate(object sender, KeyEventArgs args) {
            if (serial != null && args.KeyCode == Keys.B && InForeground())
                repeatStopUntil = clock.ElapsedMilliseconds + StopRepeatMs;
        };
    }

    private static void AddLine(FlowLayoutPanel layout, Label label, string text, int topMargin)
    {
        label.AutoSize = true;
        label.MaximumSize = new Size(460, 0);
        label.Text = text;
        label.Margin = new Padding(3, topMargin, 0, 0);
        layout.Controls.Add(label);
    }

    private void RefreshPorts()
    {
        string selected = ports.SelectedItem as string;
        string[] names = SerialPort.GetPortNames();
        Array.Sort(names, StringComparer.OrdinalIgnoreCase);
        ports.Items.Clear();
        ports.Items.AddRange(names);
        ports.SelectedIndex = Array.IndexOf(names, selected);
        if (ports.SelectedIndex < 0 && names.Length == 1)
            ports.SelectedIndex = 0;
        connect.Enabled = ports.SelectedItem != null;
        link.Text = names.Length == 0 ? "未找到 COM 口，插入串口后点刷新" : "未连接";
    }

    private void ToggleConnection()
    {
        if (serial != null)
        {
            ClosePort("已断开", true);
            return;
        }

        string name = ports.SelectedItem as string;
        if (name == null)
            return;

        var opened = new SerialPort(name, 115200, Parity.None, 8, StopBits.One) {
            Handshake = Handshake.None, WriteTimeout = 30, DtrEnable = false, RtsEnable = false
        };
        try
        {
            opened.Open();
        }
        catch (Exception error)
        {
            if (!IsSerialError(error))
                throw;
            opened.Dispose();
            link.Text = "打开 " + name + " 失败 · " + error.Message;
            return;
        }

        serial = opened;
        ports.Enabled = refresh.Enabled = false;
        connect.Text = "断开";
        acceptInput = false;
        repeatStopUntil = frameCount = 0;
        sequence = 0;
        nextFrameAt = lastWriteAt = clock.ElapsedMilliseconds;
        ClearMouse();
        SendCurrentInput();
        if (serial != null)
            timer.Start();
    }

    private void ClosePort(string message, bool sendNeutral)
    {
        timer.Stop();
        if (serial != null)
        {
            try
            {
                if (sendNeutral)
                {
                    serial.DiscardOutBuffer();
                    WriteFrame(StopOrNeutral(clock.ElapsedMilliseconds));
                }
            }
            catch (Exception error)
            {
                if (!IsSerialError(error))
                    throw;
            }
            finally
            {
                try { serial.Dispose(); }
                catch (IOException) { }
                serial = null;
            }
        }

        acceptInput = false;
        repeatStopUntil = 0;
        ClearMouse();
        ports.Enabled = refresh.Enabled = true;
        connect.Text = "连接";
        link.Text = message;
        inputStatus.Text = "输入  —";
        mouseStatus.Text = "鼠标  —";
    }

    private static bool IsSerialError(Exception error)
    {
        return error is IOException || error is InvalidOperationException ||
            error is UnauthorizedAccessException || error is TimeoutException || error is ArgumentException;
    }

    private bool InForeground()
    {
        return IsHandleCreated && GetForegroundWindow() == Handle && WindowState != FormWindowState.Minimized;
    }

    private void OnFocusLost()
    {
        acceptInput = false;
        ClearMouse();
        nextFrameAt = clock.ElapsedMilliseconds;
        SendCurrentInput(true);
    }

    private void SendCurrentInput(bool forceNeutral = false)
    {
        long now = clock.ElapsedMilliseconds;
        if (serial == null || now < nextFrameAt)
            return;

        // 调度延误后只发送当前输入，避免补发积压的操作。
        nextFrameAt += FrameIntervalMs;
        if (nextFrameAt <= now)
            nextFrameAt = now + FrameIntervalMs;
        if (now - lastWriteAt >= SendStallMs)
        {
            acceptInput = false;
            ClearMouse();
        }

        try
        {
            if (serial.BytesToWrite != 0)
            {
                if (now - lastWriteAt >= SendStallMs)
                    ClosePort("串口发送阻塞，已断开", true);
                return;
            }

            bool focused = !forceNeutral && InForeground();
            KeyMouse input = ReadInput(focused, now);
            WriteFrame(input);
            lastWriteAt = clock.ElapsedMilliseconds;
            ++frameCount;

            link.Text = serial.PortName + " 已打开 · 115200 8N1 · 已发 " + frameCount + " 帧";
            if ((input.Keys & Vt02Frame.StopKey) != 0)
                inputStatus.Text = "正在发送零力急停";
            else if (!focused)
                inputStatus.Text = "窗口失焦，发送中性帧";
            else
                inputStatus.Text = acceptInput ? "按键  " + ShowKeys(input.Keys) : "松开控制键后可操作";
            mouseStatus.Text = rawMouseReady
                ? string.Format("鼠标  X {0}   Y {1}   滚轮 {2}", input.X, input.Y, input.Wheel)
                : "鼠标位移采集不可用，可用 A/D 控制转向";
        }
        catch (Exception error)
        {
            if (!IsSerialError(error))
                throw;
            ClosePort("串口已断开 · " + error.Message, true);
        }
    }

    private KeyMouse ReadInput(bool focused, long now)
    {
        if (!focused)
        {
            acceptInput = false;
            ClearMouse();
            return StopOrNeutral(now);
        }

        var input = new KeyMouse();
        for (int i = 0; i < ProtocolKeys.Length; ++i)
            if (Down(ProtocolKeys[i]))
                input.Keys |= (ushort)(1 << i);
        input.Left = Down(0x01);
        input.Right = Down(RightMouseButton);
        if ((input.Keys & Vt02Frame.StopKey) != 0)
            repeatStopUntil = now + StopRepeatMs;
        if (now < repeatStopUntil)
        {
            acceptInput = false;
            ClearMouse();
            return StopOrNeutral(now);
        }

        if (!acceptInput)
        {
            acceptInput = input.Keys == 0;
            ClearMouse();
            return new KeyMouse();
        }

        input.X = (short)mouseX;
        input.Y = (short)mouseY;
        input.Wheel = (short)wheel;
        ClearMouse();
        return input;
    }

    private KeyMouse StopOrNeutral(long now)
    {
        return new KeyMouse { Keys = now < repeatStopUntil ? Vt02Frame.StopKey : (ushort)0 };
    }

    private void WriteFrame(KeyMouse input)
    {
        Vt02Frame.Encode(frame, sequence, input);
        serial.Write(frame, 0, frame.Length);
        sequence = unchecked((byte)(sequence + 1));
    }

    private static string ShowKeys(ushort keys)
    {
        var pressed = new System.Collections.Generic.List<string>();
        for (int i = 0; i < ProtocolKeyNames.Length; ++i)
            if ((keys & (1 << i)) != 0)
                pressed.Add(ProtocolKeyNames[i]);
        return pressed.Count != 0 ? string.Join(" + ", pressed) : "—";
    }

    private void ClearMouse()
    {
        mouseX = mouseY = wheel = 0;
    }

    private static bool Down(int key)
    {
        return (GetAsyncKeyState(key) & 0x8000) != 0;
    }

    protected override void OnHandleCreated(EventArgs args)
    {
        base.OnHandleCreated(args);
        var devices = new[] { new RawInputDevice { UsagePage = 1, Usage = 2, Target = Handle } };
        rawMouseReady = RegisterRawInputDevices(devices, 1, (uint)Marshal.SizeOf(typeof(RawInputDevice)));
    }

    protected override void WndProc(ref Message message)
    {
        if (message.Msg == WmInput && serial != null && acceptInput && InForeground())
        {
            uint expectedSize = (uint)Marshal.SizeOf(typeof(RawMouseInput));
            uint size = expectedSize;
            uint headerSize = (uint)Marshal.SizeOf(typeof(RawInputHeader));
            RawMouseInput input;
            if (GetRawInputData(message.LParam, RawInputRead, out input, ref size, headerSize) == expectedSize &&
                input.Header.Type == 0u)
            {
                if ((input.Mouse.Flags & MouseMoveAbsolute) == 0)
                {
                    mouseX = AddMouse(mouseX, input.Mouse.X);
                    mouseY = AddMouse(mouseY, -(long)input.Mouse.Y);
                }
                if ((input.Mouse.ButtonFlags & MouseWheelFlag) != 0)
                    wheel = AddMouse(wheel, input.Mouse.ButtonData);
            }
        }
        base.WndProc(ref message);
    }

    private static int AddMouse(int current, long delta)
    {
        return (int)Math.Max(short.MinValue, Math.Min(short.MaxValue, current + delta));
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct RawInputDevice
    {
        internal ushort UsagePage, Usage;
        internal uint Flags;
        internal IntPtr Target;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct RawInputHeader
    {
        internal uint Type, Size;
        internal IntPtr Device, Param;
    }

    [StructLayout(LayoutKind.Explicit, Size = 24)]
    private struct RawMouse
    {
        [FieldOffset(0)] internal ushort Flags;
        [FieldOffset(4)] internal ushort ButtonFlags;
        [FieldOffset(6)] internal short ButtonData;
        [FieldOffset(12)] internal int X;
        [FieldOffset(16)] internal int Y;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct RawMouseInput
    {
        internal RawInputHeader Header;
        internal RawMouse Mouse;
    }

    [DllImport("user32.dll")]
    private static extern IntPtr GetForegroundWindow();

    [DllImport("user32.dll")]
    private static extern short GetAsyncKeyState(int key);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool RegisterRawInputDevices(RawInputDevice[] devices, uint count, uint size);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern uint GetRawInputData(IntPtr handle, uint command, out RawMouseInput input,
        ref uint size, uint headerSize);
}

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new TeleopWindow());
    }
}
