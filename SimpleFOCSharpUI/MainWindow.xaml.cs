using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Threading;
using System.Management;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Threading;
using System.Collections.Concurrent;

namespace SimpleFOCSharpUI
{
    public partial class MainWindow : Window
    {
        private SerialPortManager _serialManager;
        private readonly Queue<MotorStatus> _motorStatusHistory = new Queue<MotorStatus>();
        private const int MAX_HISTORY_COUNT = 1000;
        private bool _isConnected = false;
        private readonly ConcurrentQueue<string> _terminalBuffer = new ConcurrentQueue<string>();
        private const int MAX_TERMINAL_LINES = 1000;  // Limit line count to prevent memory issues
        private readonly DispatcherTimer _terminalUpdateTimer;
        private volatile bool _isProcessingTerminal = false;

        private class MotorStatus
        {
            public double Time { get; set; }
            public double Target { get; set; }
            public double Position { get; set; }
            public double Error { get; set; }
            public double Velocity { get; set; }
        }

        public MainWindow()
        {
            InitializeComponent();
            _serialManager = new SerialPortManager();
            SetupSerialEvents();
            InitializeUI();
            Loaded += Window_Loaded;
            _terminalUpdateTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)  // Update terminal every 100ms
            };
            _terminalUpdateTimer.Tick += ProcessTerminalBuffer;
            _terminalUpdateTimer.Start();
        }
        private void UpdateTerminal(string message)
        {
            _terminalBuffer.Enqueue($"{DateTime.Now:HH:mm:ss.fff} > {message}");
        }

        private void InitializeUI()
        {
            // Initialize ComboBox items
            foreach (string port in SerialPort.GetPortNames())
            {
                ComPortSelect.Items.Add(port);
            }
            if (ComPortSelect.Items.Count > 0)
                ComPortSelect.SelectedIndex = 0;

            // Initialize control states
            DisconnectButton.IsEnabled = false;

            // Add baud rates if needed
            if (!ComPortSelect.Items.Contains("115200"))
                ComPortSelect.Items.Add("115200");
        }

        private void SetupSerialEvents()
        {
            _serialManager.DataReceived += (s, data) =>
            {
                Dispatcher.Invoke(() =>
                {
                    ProcessReceivedData(data);
                    UpdateTerminal(data);
                });
            };

            _serialManager.ConnectionStatusChanged += (s, status) =>
            {
                Dispatcher.Invoke(() =>
                {
                    UpdateConnectionStatus(status);
                });
            };
        }

        private void ProcessTerminalBuffer(object sender, EventArgs e)
        {
            if (_isProcessingTerminal) return;
            _isProcessingTerminal = true;

            try
            {
                if (_terminalBuffer.IsEmpty) return;

                // Create a new paragraph for batch update
                var paragraph = new Paragraph();
                int processedCount = 0;

                // Process up to 100 messages at once
                while (_terminalBuffer.TryDequeue(out string message) && processedCount < 100)
                {
                    paragraph.Inlines.Add(new Run(message + Environment.NewLine));
                    processedCount++;
                }

                // Add new content to the document
                var document = dataTextBox.Document;
                document.Blocks.Add(paragraph);

                // Remove old lines if exceeding maximum
                while (document.Blocks.Count > MAX_TERMINAL_LINES)
                {
                    document.Blocks.Remove(document.Blocks.FirstBlock);
                }

                // Auto-scroll if near bottom
                if (Math.Abs(dataTextBox.VerticalOffset - dataTextBox.ExtentHeight) < dataTextBox.ViewportHeight + 50)
                {
                    dataTextBox.ScrollToEnd();
                }
            }
            finally
            {
                _isProcessingTerminal = false;
            }
        }

        // Cleanup
        protected async override void OnClosing(CancelEventArgs e)
        {
            _terminalUpdateTimer.Stop();
            await _serialManager.DisconnectAsync();
            base.OnClosing(e);
        }
        private void ProcessReceivedData(string data)
        {
            try
            {
                // Check if it's a parameter response (contains ':')
                if (data.Contains(":"))
                {
                    var parts = data.Split(':');
                    if (parts.Length == 2)
                    {
                        string parameter = parts[0].Trim();

                        if (double.TryParse(parts[1], out double value))
                        {
                            switch (parameter.ToUpper())
                            {
                                // Position PID
                                case "P":
                                    PositionP.Text = value.ToString("F3");
                                    UpdateTerminal($"Position P gain: {value:F3}");
                                    break;
                                case "I":
                                    PositionI.Text = value.ToString("F3");
                                    UpdateTerminal($"Position I gain: {value:F3}");
                                    break;
                                case "D":
                                    PositionD.Text = value.ToString("F3");
                                    UpdateTerminal($"Position D gain: {value:F3}");
                                    break;

                                // Velocity PID
                                case "K":
                                    VelocityP.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity P gain: {value:F3}");
                                    break;
                                case "L":
                                    VelocityI.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity I gain: {value:F3}");
                                    break;
                                case "N":
                                    VelocityD.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity D gain: {value:F3}");
                                    break;

                                // Motor limits
                                case "V":
                                    VoltageLimit.Text = value.ToString("F3");
                                    UpdateTerminal($"Voltage limit: {value:F3}V");
                                    break;
                                case "C":
                                    CurrentLimit.Text = value.ToString("F3");
                                    UpdateTerminal($"Current limit: {value:F3}A");
                                    break;
                                case "S":
                                    VelocityLimit.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity limit: {value:F3}rad/s");
                                    break;

                                // Motor parameters
                                case "R":
                                    VelocityRamp.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity ramp: {value:F3}");
                                    break;
                                case "F":
                                    VelocityFilter.Text = value.ToString("F3");
                                    UpdateTerminal($"Velocity filter Tf: {value:F3}");
                                    break;
                                case "U":
                                    SupplyVoltage.Text = value.ToString("F3");
                                    UpdateTerminal($"Supply voltage: {value:F3}V");
                                    break;

                                // Control modes
                                case "Y":
                                    int controlMode = (int)value;
                                    if (controlMode >= 0 && controlMode < ControlMode.Items.Count)
                                    {
                                        ControlMode.SelectedIndex = controlMode;
                                        string modeName = ((ComboBoxItem)ControlMode.Items[controlMode]).Content.ToString();
                                        UpdateTerminal($"Control mode: {modeName}");
                                    }
                                    break;

                                case "Z":
                                    int torqueMode = (int)value;
                                    if (torqueMode >= 0 && torqueMode < TorqueMode.Items.Count)
                                    {
                                        TorqueMode.SelectedIndex = torqueMode;
                                        string modeName = ((ComboBoxItem)TorqueMode.Items[torqueMode]).Content.ToString();
                                        UpdateTerminal($"Torque mode: {modeName}");
                                    }
                                    break;
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Error processing data: {ex.Message}");
                UpdateTerminal($"Error processing data: {ex.Message}");
            }
        }

        private void UpdateConnectionStatus(string status)
        {
            _isConnected = status.Contains("Connected");
            ConnectButton.IsEnabled = !_isConnected;
            DisconnectButton.IsEnabled = _isConnected;
            ComPortSelect.IsEnabled = !_isConnected;
        }

        private async void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (ComPortSelect.SelectedItem == null) return;

            string selectedPort = ComPortSelect.SelectedItem.ToString();
            int baudRate = 115200; // Default baud rate for SimpleFOC

            var (success, error) = await _serialManager.ConnectAsync(selectedPort, baudRate);
            if (!success)
            {
                MessageBox.Show($"Connection failed: {error}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private async void DisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            await _serialManager.DisconnectAsync();
        }

        private async void SendCommand(string command)
        {
            if (!_isConnected) return;

            try
            {
                await _serialManager.SendDataAsync(command);
                UpdateTerminal($"Sent: {command}");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending command: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        #region Command Methods
        private void SetPositionP_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(PositionP.Text, out double value))
                SendCommand($"P{value}");
        }

        private void SetPositionI_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(PositionI.Text, out double value))
                SendCommand($"I{value}");
        }

        private void SetPositionD_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(PositionD.Text, out double value))
                SendCommand($"D{value}");
        }

        private void SetVelocityP_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityP.Text, out double value))
                SendCommand($"K{value}");
        }

        private void SetVelocityI_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityI.Text, out double value))
                SendCommand($"L{value}");
        }

        private void SetVelocityD_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityD.Text, out double value))
                SendCommand($"N{value}");  // Changed from M to N
        }

        private void SetVoltageLimit_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VoltageLimit.Text, out double value))
                SendCommand($"V{value}");
        }

        private void SetCurrentLimit_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(CurrentLimit.Text, out double value))
                SendCommand($"C{value}");
        }

        private void SetVelocityLimit_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityLimit.Text, out double value))
                SendCommand($"S{value}");  // Changed to S for velocity limit
        }

        private void SetControlMode_Click(object sender, RoutedEventArgs e)
        {
            var selectedItem = ControlMode.SelectedItem as ComboBoxItem;
            if (selectedItem != null)
            {
                int modeValue = ControlMode.SelectedIndex;
                SendCommand($"Y{modeValue}");  // Changed to Y for controller type
            }
        }

        private void SetTorqueMode_Click(object sender, RoutedEventArgs e)
        {
            var selectedItem = TorqueMode.SelectedItem as ComboBoxItem;
            if (selectedItem != null)
            {
                int modeValue = TorqueMode.SelectedIndex;
                SendCommand($"Z{modeValue}");  // Changed to Z for torque type
            }
        }
        private void SetVelocityRamp_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityRamp.Text, out double value))
                SendCommand($"R{value}");
        }

        private void SetVelocityFilter_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(VelocityFilter.Text, out double value))
                SendCommand($"F{value}");
        }

        private void SetSupplyVoltage_Click(object sender, RoutedEventArgs e)
        {
            if (double.TryParse(SupplyVoltage.Text, out double value))
                SendCommand($"U{value}");
        }

        // Additional refresh handlers
        private void RefreshVelocityRamp_Click(object sender, RoutedEventArgs e)
        {
            SendCommand("R");
        }

        private void RefreshVelocityFilter_Click(object sender, RoutedEventArgs e)
        {
            SendCommand("F");
        }

        private void RefreshSupplyVoltage_Click(object sender, RoutedEventArgs e)
        {
            SendCommand("U");
        }
        #endregion
        private void TerminalSend_Click(object sender, RoutedEventArgs e)
        {
            SendTerminalCommand();
        }

        private void TerminalInput_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendTerminalCommand();
            }
        }

        private async void SendTerminalCommand()
        {
            if (string.IsNullOrWhiteSpace(terminalInput.Text)) return;

            try
            {
                await _serialManager.SendDataAsync(terminalInput.Text);
                terminalInput.Clear();
            }
            catch (Exception ex)
            {
                UpdateTerminal($"Error sending command: {ex.Message}");
            }
        }

        // Refresh commands - just send the letter
        private void RefreshPositionP_Click(object sender, RoutedEventArgs e) => SendCommand("P");
        private void RefreshPositionI_Click(object sender, RoutedEventArgs e) => SendCommand("I");
        private void RefreshPositionD_Click(object sender, RoutedEventArgs e) => SendCommand("D");
        private void RefreshVelocityP_Click(object sender, RoutedEventArgs e) => SendCommand("K");
        private void RefreshVelocityI_Click(object sender, RoutedEventArgs e) => SendCommand("L");
        private void RefreshVelocityD_Click(object sender, RoutedEventArgs e) => SendCommand("N");
        private void RefreshVoltageLimit_Click(object sender, RoutedEventArgs e) => SendCommand("V");
        private void RefreshCurrentLimit_Click(object sender, RoutedEventArgs e) => SendCommand("C");
        private void RefreshVelocityLimit_Click(object sender, RoutedEventArgs e) => SendCommand("S");
        private void RefreshControlMode_Click(object sender, RoutedEventArgs e) => SendCommand("Y");
        private void RefreshTorqueMode_Click(object sender, RoutedEventArgs e) => SendCommand("Z");



        private async void ResetESP32_Click(object sender, RoutedEventArgs e)
        {
            await _serialManager.ResetDeviceAsync();
            UpdateTerminal("Device reset requested");
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            var (success, ports) = await _serialManager.GetAvailablePortsAsync();
            if (success)
            {
                ComPortSelect.Items.Clear();
                foreach (string port in ports)
                {
                    ComPortSelect.Items.Add(port);
                }
                if (ComPortSelect.Items.Count > 0)
                    ComPortSelect.SelectedIndex = 0;
            }
        }

        private async void ReadAllParameters_Click(object sender, RoutedEventArgs e)
        {
            if (!_isConnected) return;

            string[] parameters = { "P", "I", "D", "K", "L", "N", "V", "C", "S", "Y", "Z", "R", "F", "U" };
            foreach (var param in parameters)
            {
                await _serialManager.SendDataAsync(param);
                await Task.Delay(100);
            }
            UpdateTerminal("Reading all parameters...");
        }

    }
}