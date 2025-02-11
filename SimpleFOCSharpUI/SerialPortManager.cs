using System;
using System.Collections.Concurrent;
using System.IO.Ports;
using System.Management;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Linq;

public class SerialPortManager : IDisposable
{
    private SerialPort _serialPort;
    private readonly SemaphoreSlim _serialPortLock = new SemaphoreSlim(1, 1);
    private ManagementEventWatcher _portWatcher;
    private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();
    private readonly StringBuilder _messageBuffer = new StringBuilder();
    private CancellationTokenSource _closingCts;

    public event EventHandler<string> DataReceived;
    public event EventHandler<string> ConnectionStatusChanged;
    public event EventHandler<string> PortsChanged;

    public bool IsConnected => _serialPort?.IsOpen ?? false;

    public SerialPortManager()
    {
        _closingCts = new CancellationTokenSource();
        SetupPortWatcher();
    }

    private void SetupPortWatcher()
    {
        try
        {
            var query = new WqlEventQuery("SELECT * FROM Win32_DeviceChangeEvent WHERE EventType = 2 OR EventType = 3");
            _portWatcher = new ManagementEventWatcher(query);
            _portWatcher.EventArrived += PortWatcher_EventArrived;
            _portWatcher.Start();
        }
        catch (Exception ex)
        {
            ConnectionStatusChanged?.Invoke(this, $"Port monitoring error: {ex.Message}");
        }
    }

    private void PortWatcher_EventArrived(object sender, EventArrivedEventArgs e)
    {
        if (_serialPort?.IsOpen == true && !IsPortAvailable(_serialPort.PortName))
        {
            DisconnectAsync().Wait();
            ConnectionStatusChanged?.Invoke(this, "USB connection lost!");
        }

        PortsChanged?.Invoke(this, "Ports changed");
    }

    public async Task<(bool success, string[] ports)> GetAvailablePortsAsync()
    {
        try
        {
            var ports = await Task.Run(() => SerialPort.GetPortNames().OrderBy(x => x).ToArray());
            return (true, ports);
        }
        catch (Exception ex)
        {
            return (false, Array.Empty<string>());
        }
    }

    public async Task<(bool success, string error)> ConnectAsync(string portName, int baudRate)
    {
        if (_serialPort?.IsOpen == true)
            return (false, "Already connected");

        SerialPort tempPort = null;
        try
        {
            await _serialPortLock.WaitAsync();

            tempPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                DtrEnable = false,
                RtsEnable = false,
                WriteTimeout = 500,
                ReadTimeout = -1
            };


            using (CancellationTokenSource timeoutCts = new CancellationTokenSource(TimeSpan.FromSeconds(3)))
            {
                using (CancellationTokenSource linkedCts = CancellationTokenSource.CreateLinkedTokenSource(timeoutCts.Token, _closingCts.Token))
                {

                    bool success = await Task.Run(async () =>
                    {
                        try
                        {
                            linkedCts.Token.ThrowIfCancellationRequested();
                            tempPort.Open();
                            return true;
                        }
                        catch
                        {
                            return false;
                        }
                    }, linkedCts.Token);

                    if (success)
                    {
                        tempPort.DataReceived += SerialPort_DataReceived;
                        _serialPort = tempPort;
                        ConnectionStatusChanged?.Invoke(this, "Connected");
                        return (true, null);
                    }
                    else
                    {
                        await SafeClosePort(tempPort);
                        return (false, "Could not open port");
                    }
                }
            }

        }
        catch (Exception ex)
        {
            await SafeClosePort(tempPort);
            return (false, ex.Message);
        }
        finally
        {
            _serialPortLock.Release();
        }
    }

    private async Task SafeClosePort(SerialPort port)
    {
        if (port != null)
        {
            try
            {
                if (port.IsOpen)
                {
                    port.RtsEnable = false;
                    port.DtrEnable = false;
                    await Task.Delay(100);
                    port.Close();
                }
                port.Dispose();
            }
            catch { }
        }
    }

    public async Task DisconnectAsync()
    {
        if (_serialPort == null) return;

        try
        {
            await _serialPortLock.WaitAsync();
            try
            {
                _serialPort.DataReceived -= SerialPort_DataReceived;
                await SafeClosePort(_serialPort);
                _serialPort = null;

                await Task.Delay(500);
                ConnectionStatusChanged?.Invoke(this, "Disconnected");
            }
            finally
            {
                _serialPortLock.Release();
            }
        }
        catch (Exception ex)
        {
            ConnectionStatusChanged?.Invoke(this, $"Disconnect error: {ex.Message}");
        }
    }

    private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        if (_serialPort == null) return;

        try
        {
            _serialPortLock.Wait();
            try
            {
                if (!_serialPort.IsOpen) return;

                string data = Task.Run(() =>
                {
                    try
                    {
                        _serialPort.Encoding = Encoding.UTF8;
                        return _serialPort.ReadExisting();
                    }
                    catch (TimeoutException)
                    {
                        return string.Empty;
                    }
                }).Result;

                if (!string.IsNullOrEmpty(data))
                {
                    _messageBuffer.Append(data);
                    string bufferStr = _messageBuffer.ToString();
                    int newlineIndex;

                    while ((newlineIndex = bufferStr.IndexOf('\n')) != -1)
                    {
                        string line = bufferStr.Substring(0, newlineIndex).Trim();
                        if (!string.IsNullOrEmpty(line))
                        {
                            DataReceived?.Invoke(this, line);
                        }
                        bufferStr = bufferStr.Substring(newlineIndex + 1);
                    }

                    _messageBuffer.Clear();
                    _messageBuffer.Append(bufferStr);
                }
            }
            finally
            {
                _serialPortLock.Release();
            }
        }
        catch (Exception ex)
        {
            if (ex is System.IO.IOException || ex is InvalidOperationException)
            {
                DisconnectAsync().Wait();
            }
        }
    }

    public async Task<bool> SendDataAsync(string data)
    {
        if (_serialPort == null || !_serialPort.IsOpen) return false;

        try
        {
            await _serialPortLock.WaitAsync();
            try
            {
                return await Task.Run(() =>
                {
                    try
                    {
                        _serialPort.WriteLine(data);
                        return true;
                    }
                    catch
                    {
                        return false;
                    }
                });
            }
            finally
            {
                _serialPortLock.Release();
            }
        }
        catch
        {
            return false;
        }
    }

    public async Task ResetDeviceAsync()
    {
        if (_serialPort == null || !_serialPort.IsOpen) return;

        try
        {
            await _serialPortLock.WaitAsync();
            try
            {
                _serialPort.RtsEnable = true;
                _serialPort.DtrEnable = false;
                await Task.Delay(50);

                _serialPort.RtsEnable = true;
                _serialPort.DtrEnable = true;
                await Task.Delay(50);

                _serialPort.RtsEnable = false;
                await Task.Delay(50);

                _serialPort.RtsEnable = true;
                _serialPort.DtrEnable = true;
            }
            finally
            {
                _serialPortLock.Release();
            }
        }
        catch (Exception ex)
        {
            ConnectionStatusChanged?.Invoke(this, $"Reset error: {ex.Message}");
        }
    }

    private bool IsPortAvailable(string portName)
    {
        try
        {
            return SerialPort.GetPortNames().Contains(portName);
        }
        catch
        {
            return false;
        }
    }

    public void Dispose()
    {
        _closingCts?.Cancel();
        _portWatcher?.Stop();
        _portWatcher?.Dispose();
        DisconnectAsync().Wait();
        _closingCts?.Dispose();
        _serialPortLock?.Dispose();
    }
}