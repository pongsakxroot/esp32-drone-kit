import React, { useState, useEffect, useRef } from 'react';
import mqtt from 'mqtt';
import './App.css';

// Chart.js imports for telemetry visualization
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from 'chart.js';
import { Line } from 'react-chartjs-2';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

const App = () => {
  // MQTT Configuration
  const [mqttConfig] = useState({
    broker: 'ws://192.168.1.100:8080', // WebSocket MQTT broker
    options: {
      clientId: 'react_drone_dashboard',
      username: 'drone_dashboard',
      password: 'drone_password',
    }
  });

  // Connection state
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const mqttClient = useRef(null);

  // Drone state
  const [droneStatus, setDroneStatus] = useState({
    armed: false,
    emergency: false,
    sensorsReady: false,
    wifiConnected: false,
    mqttConnected: false,
    uptime: 0,
    freeHeap: 0,
    wifiRssi: 0,
  });

  // Telemetry data
  const [telemetryData, setTelemetryData] = useState({
    timestamp: 0,
    imu: {
      accel: { x: 0, y: 0, z: 0 },
      gyro: { x: 0, y: 0, z: 0 },
      temperature: 0,
    },
    motors: [1000, 1000, 1000, 1000],
    battery: { voltage: 0, percentage: 0 },
    pid: { roll_setpoint: 0, pitch_setpoint: 0, yaw_setpoint: 0 },
  });

  // Chart data for real-time telemetry
  const [chartData, setChartData] = useState({
    labels: [],
    accelData: { x: [], y: [], z: [] },
    gyroData: { x: [], y: [], z: [] },
    batteryData: [],
    motorData: [[], [], [], []],
  });

  // Control inputs
  const [controlInputs, setControlInputs] = useState({
    throttle: 0,
    rollSetpoint: 0,
    pitchSetpoint: 0,
    yawSetpoint: 0,
  });

  // UI state
  const [activeTab, setActiveTab] = useState('overview');
  const [alertMessages, setAlertMessages] = useState([]);
  const maxDataPoints = 50;

  useEffect(() => {
    connectToMQTT();
    return () => {
      if (mqttClient.current) {
        mqttClient.current.end();
      }
    };
  }, []);

  const connectToMQTT = () => {
    setConnectionStatus('Connecting...');
    
    try {
      mqttClient.current = mqtt.connect(mqttConfig.broker, mqttConfig.options);

      mqttClient.current.on('connect', () => {
        setIsConnected(true);
        setConnectionStatus('Connected');
        console.log('MQTT Connected');
        
        // Subscribe to drone topics
        mqttClient.current.subscribe('drone/telemetry');
        mqttClient.current.subscribe('drone/status');
        mqttClient.current.subscribe('drone/emergency');
        
        addAlert('Connected to drone', 'success');
      });

      mqttClient.current.on('message', (topic, message) => {
        try {
          const data = JSON.parse(message.toString());
          handleMQTTMessage(topic, data);
        } catch (error) {
          console.error('Error parsing MQTT message:', error);
        }
      });

      mqttClient.current.on('error', (error) => {
        console.error('MQTT Error:', error);
        setConnectionStatus('Error');
        addAlert('MQTT connection error', 'error');
      });

      mqttClient.current.on('close', () => {
        setIsConnected(false);
        setConnectionStatus('Disconnected');
        addAlert('Disconnected from drone', 'warning');
      });

    } catch (error) {
      console.error('Failed to connect to MQTT:', error);
      setConnectionStatus('Failed to connect');
      addAlert('Failed to connect to drone', 'error');
    }
  };

  const handleMQTTMessage = (topic, data) => {
    const timestamp = new Date().toLocaleTimeString();
    
    if (topic === 'drone/telemetry') {
      setTelemetryData(data);
      updateChartData(data, timestamp);
    } else if (topic === 'drone/status') {
      setDroneStatus(data);
    } else if (topic === 'drone/emergency') {
      addAlert(`Emergency: ${data.reason || 'Unknown'}`, 'error');
    }
  };

  const updateChartData = (data, timestamp) => {
    setChartData(prevData => {
      const newLabels = [...prevData.labels, timestamp].slice(-maxDataPoints);
      
      return {
        labels: newLabels,
        accelData: {
          x: [...prevData.accelData.x, data.imu?.accel?.x || 0].slice(-maxDataPoints),
          y: [...prevData.accelData.y, data.imu?.accel?.y || 0].slice(-maxDataPoints),
          z: [...prevData.accelData.z, data.imu?.accel?.z || 0].slice(-maxDataPoints),
        },
        gyroData: {
          x: [...prevData.gyroData.x, data.imu?.gyro?.x || 0].slice(-maxDataPoints),
          y: [...prevData.gyroData.y, data.imu?.gyro?.y || 0].slice(-maxDataPoints),
          z: [...prevData.gyroData.z, data.imu?.gyro?.z || 0].slice(-maxDataPoints),
        },
        batteryData: [...prevData.batteryData, data.battery?.percentage || 0].slice(-maxDataPoints),
        motorData: [
          [...prevData.motorData[0], data.motors?.[0] || 1000].slice(-maxDataPoints),
          [...prevData.motorData[1], data.motors?.[1] || 1000].slice(-maxDataPoints),
          [...prevData.motorData[2], data.motors?.[2] || 1000].slice(-maxDataPoints),
          [...prevData.motorData[3], data.motors?.[3] || 1000].slice(-maxDataPoints),
        ],
      };
    });
  };

  const sendControlCommand = (command) => {
    if (!isConnected) {
      addAlert('Not connected to drone', 'warning');
      return;
    }

    mqttClient.current.publish('drone/control', JSON.stringify(command));
  };

  const sendEmergencyCommand = (command) => {
    if (!isConnected) return;
    mqttClient.current.publish('drone/emergency', JSON.stringify(command));
  };

  const addAlert = (message, type) => {
    const alert = {
      id: Date.now(),
      message,
      type,
      timestamp: new Date().toLocaleTimeString(),
    };
    
    setAlertMessages(prev => [alert, ...prev.slice(0, 9)]); // Keep last 10 alerts
    
    // Auto-remove success alerts after 3 seconds
    if (type === 'success') {
      setTimeout(() => {
        setAlertMessages(prev => prev.filter(a => a.id !== alert.id));
      }, 3000);
    }
  };

  const armMotors = () => {
    sendControlCommand({ command: 'arm' });
    addAlert('Arming motors...', 'info');
  };

  const disarmMotors = () => {
    sendControlCommand({ command: 'disarm' });
    addAlert('Disarming motors...', 'info');
  };

  const emergencyStop = () => {
    sendEmergencyCommand({ command: 'stop' });
    addAlert('EMERGENCY STOP ACTIVATED', 'error');
  };

  const resetEmergency = () => {
    sendEmergencyCommand({ command: 'reset' });
    addAlert('Emergency reset', 'info');
  };

  const updateThrottle = (value) => {
    setControlInputs(prev => ({ ...prev, throttle: value }));
    sendControlCommand({ command: 'throttle', value: parseInt(value) });
  };

  const updateSetpoints = (roll, pitch, yaw) => {
    setControlInputs(prev => ({
      ...prev,
      rollSetpoint: roll,
      pitchSetpoint: pitch,
      yawSetpoint: yaw,
    }));
    
    sendControlCommand({
      command: 'setpoint',
      roll: parseFloat(roll),
      pitch: parseFloat(pitch),
      yaw: parseFloat(yaw),
    });
  };

  const getStatusColor = (status) => {
    if (status) return '#4CAF50';
    return '#f44336';
  };

  const getBatteryColor = (percentage) => {
    if (percentage > 50) return '#4CAF50';
    if (percentage > 20) return '#FF9800';
    return '#f44336';
  };

  const renderOverviewTab = () => (
    <div className="overview-grid">
      {/* Connection Status */}
      <div className="status-card">
        <h3>Connection Status</h3>
        <div className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
          {connectionStatus}
        </div>
        {!isConnected && (
          <button onClick={connectToMQTT} className="btn btn-primary">
            Reconnect
          </button>
        )}
      </div>

      {/* Drone Status */}
      <div className="status-card">
        <h3>Drone Status</h3>
        <div className="status-grid">
          <div className="status-item">
            <span>Armed:</span>
            <span style={{ color: getStatusColor(droneStatus.armed) }}>
              {droneStatus.armed ? 'YES' : 'NO'}
            </span>
          </div>
          <div className="status-item">
            <span>Sensors:</span>
            <span style={{ color: getStatusColor(droneStatus.sensorsReady) }}>
              {droneStatus.sensorsReady ? 'READY' : 'NOT READY'}
            </span>
          </div>
          <div className="status-item">
            <span>Emergency:</span>
            <span style={{ color: getStatusColor(!droneStatus.emergency) }}>
              {droneStatus.emergency ? 'ACTIVE' : 'CLEAR'}
            </span>
          </div>
          <div className="status-item">
            <span>WiFi RSSI:</span>
            <span>{droneStatus.wifiRssi} dBm</span>
          </div>
        </div>
      </div>

      {/* Battery Status */}
      <div className="status-card">
        <h3>Battery</h3>
        <div className="battery-display">
          <div className="battery-percentage">
            {telemetryData.battery?.percentage?.toFixed(0) || 0}%
          </div>
          <div className="battery-voltage">
            {telemetryData.battery?.voltage?.toFixed(2) || 0}V
          </div>
          <div className="battery-bar">
            <div 
              className="battery-fill"
              style={{ 
                width: `${telemetryData.battery?.percentage || 0}%`,
                backgroundColor: getBatteryColor(telemetryData.battery?.percentage || 0)
              }}
            />
          </div>
        </div>
      </div>

      {/* Motor Status */}
      <div className="status-card">
        <h3>Motors</h3>
        <div className="motor-grid">
          {['FL', 'FR', 'BL', 'BR'].map((label, index) => {
            const motorValue = telemetryData.motors?.[index] || 1000;
            const percentage = ((motorValue - 1000) / 10).toFixed(1);
            
            return (
              <div key={label} className="motor-indicator">
                <div className="motor-label">{label}</div>
                <div className="motor-value">{motorValue}μs</div>
                <div className="motor-bar">
                  <div 
                    className="motor-fill"
                    style={{ 
                      width: `${Math.min(percentage, 100)}%`,
                      backgroundColor: percentage > 50 ? '#f44336' : '#4CAF50'
                    }}
                  />
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* IMU Data */}
      <div className="status-card">
        <h3>IMU Sensors</h3>
        <div className="imu-grid">
          <div className="imu-section">
            <h4>Accelerometer (g)</h4>
            <div>X: {telemetryData.imu?.accel?.x?.toFixed(2) || 0}</div>
            <div>Y: {telemetryData.imu?.accel?.y?.toFixed(2) || 0}</div>
            <div>Z: {telemetryData.imu?.accel?.z?.toFixed(2) || 0}</div>
          </div>
          <div className="imu-section">
            <h4>Gyroscope (°/s)</h4>
            <div>X: {telemetryData.imu?.gyro?.x?.toFixed(1) || 0}</div>
            <div>Y: {telemetryData.imu?.gyro?.y?.toFixed(1) || 0}</div>
            <div>Z: {telemetryData.imu?.gyro?.z?.toFixed(1) || 0}</div>
          </div>
          <div className="imu-section">
            <h4>Temperature</h4>
            <div>{telemetryData.imu?.temperature?.toFixed(1) || 0}°C</div>
          </div>
        </div>
      </div>

      {/* System Info */}
      <div className="status-card">
        <h3>System Info</h3>
        <div className="system-info">
          <div>Uptime: {Math.floor((droneStatus.uptime || 0) / 60)}m {(droneStatus.uptime || 0) % 60}s</div>
          <div>Free Heap: {Math.floor((droneStatus.freeHeap || 0) / 1024)}KB</div>
          <div>Last Update: {new Date(telemetryData.timestamp || 0).toLocaleTimeString()}</div>
        </div>
      </div>
    </div>
  );

  const renderControlTab = () => (
    <div className="control-section">
      {/* Emergency Controls */}
      <div className="control-card emergency-controls">
        <h3>Emergency Controls</h3>
        <div className="emergency-buttons">
          <button 
            onClick={emergencyStop}
            className="btn btn-emergency"
          >
            🚨 EMERGENCY STOP
          </button>
          {droneStatus.emergency && (
            <button 
              onClick={resetEmergency}
              className="btn btn-warning"
            >
              Reset Emergency
            </button>
          )}
        </div>
      </div>

      {/* Arm/Disarm Controls */}
      <div className="control-card">
        <h3>Motor Control</h3>
        <div className="arm-controls">
          <button 
            onClick={armMotors}
            disabled={droneStatus.armed || !droneStatus.sensorsReady}
            className="btn btn-success"
          >
            ARM MOTORS
          </button>
          <button 
            onClick={disarmMotors}
            disabled={!droneStatus.armed}
            className="btn btn-warning"
          >
            DISARM MOTORS
          </button>
        </div>
      </div>

      {/* Throttle Control */}
      <div className="control-card">
        <h3>Throttle Control</h3>
        <div className="throttle-control">
          <label>Throttle: {controlInputs.throttle}%</label>
          <input
            type="range"
            min="0"
            max="100"
            value={controlInputs.throttle}
            onChange={(e) => updateThrottle(e.target.value)}
            disabled={!droneStatus.armed || droneStatus.emergency}
            className="slider"
          />
        </div>
      </div>

      {/* Attitude Setpoints */}
      <div className="control-card">
        <h3>Attitude Setpoints</h3>
        <div className="setpoint-controls">
          <div className="setpoint-control">
            <label>Roll: {controlInputs.rollSetpoint}°</label>
            <input
              type="range"
              min="-30"
              max="30"
              step="0.5"
              value={controlInputs.rollSetpoint}
              onChange={(e) => updateSetpoints(e.target.value, controlInputs.pitchSetpoint, controlInputs.yawSetpoint)}
              className="slider"
            />
          </div>
          
          <div className="setpoint-control">
            <label>Pitch: {controlInputs.pitchSetpoint}°</label>
            <input
              type="range"
              min="-30"
              max="30"
              step="0.5"
              value={controlInputs.pitchSetpoint}
              onChange={(e) => updateSetpoints(controlInputs.rollSetpoint, e.target.value, controlInputs.yawSetpoint)}
              className="slider"
            />
          </div>
          
          <div className="setpoint-control">
            <label>Yaw Rate: {controlInputs.yawSetpoint}°/s</label>
            <input
              type="range"
              min="-180"
              max="180"
              step="5"
              value={controlInputs.yawSetpoint}
              onChange={(e) => updateSetpoints(controlInputs.rollSetpoint, controlInputs.pitchSetpoint, e.target.value)}
              className="slider"
            />
          </div>
        </div>
      </div>
    </div>
  );

  const renderChartsTab = () => {
    const chartOptions = {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          display: false,
        },
        y: {
          beginAtZero: false,
        },
      },
      plugins: {
        legend: {
          position: 'top',
        },
      },
      animation: {
        duration: 0,
      },
    };

    const accelChartData = {
      labels: chartData.labels,
      datasets: [
        {
          label: 'Accel X',
          data: chartData.accelData.x,
          borderColor: 'rgb(255, 99, 132)',
          backgroundColor: 'rgba(255, 99, 132, 0.2)',
        },
        {
          label: 'Accel Y',
          data: chartData.accelData.y,
          borderColor: 'rgb(54, 162, 235)',
          backgroundColor: 'rgba(54, 162, 235, 0.2)',
        },
        {
          label: 'Accel Z',
          data: chartData.accelData.z,
          borderColor: 'rgb(75, 192, 192)',
          backgroundColor: 'rgba(75, 192, 192, 0.2)',
        },
      ],
    };

    const gyroChartData = {
      labels: chartData.labels,
      datasets: [
        {
          label: 'Gyro X',
          data: chartData.gyroData.x,
          borderColor: 'rgb(255, 205, 86)',
          backgroundColor: 'rgba(255, 205, 86, 0.2)',
        },
        {
          label: 'Gyro Y',
          data: chartData.gyroData.y,
          borderColor: 'rgb(153, 102, 255)',
          backgroundColor: 'rgba(153, 102, 255, 0.2)',
        },
        {
          label: 'Gyro Z',
          data: chartData.gyroData.z,
          borderColor: 'rgb(255, 159, 64)',
          backgroundColor: 'rgba(255, 159, 64, 0.2)',
        },
      ],
    };

    const batteryChartData = {
      labels: chartData.labels,
      datasets: [
        {
          label: 'Battery %',
          data: chartData.batteryData,
          borderColor: 'rgb(75, 192, 192)',
          backgroundColor: 'rgba(75, 192, 192, 0.2)',
        },
      ],
    };

    return (
      <div className="charts-section">
        <div className="chart-card">
          <h3>Accelerometer Data (g)</h3>
          <div className="chart-container">
            <Line data={accelChartData} options={chartOptions} />
          </div>
        </div>
        
        <div className="chart-card">
          <h3>Gyroscope Data (°/s)</h3>
          <div className="chart-container">
            <Line data={gyroChartData} options={chartOptions} />
          </div>
        </div>
        
        <div className="chart-card">
          <h3>Battery Level (%)</h3>
          <div className="chart-container">
            <Line data={batteryChartData} options={chartOptions} />
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className="app">
      <header className="app-header">
        <h1>🚁 ESP32 Drone Dashboard</h1>
        <div className="connection-indicator">
          <div className={`indicator ${isConnected ? 'connected' : 'disconnected'}`} />
          <span>{connectionStatus}</span>
        </div>
      </header>

      <nav className="app-nav">
        <button 
          className={`nav-btn ${activeTab === 'overview' ? 'active' : ''}`}
          onClick={() => setActiveTab('overview')}
        >
          Overview
        </button>
        <button 
          className={`nav-btn ${activeTab === 'control' ? 'active' : ''}`}
          onClick={() => setActiveTab('control')}
        >
          Control
        </button>
        <button 
          className={`nav-btn ${activeTab === 'charts' ? 'active' : ''}`}
          onClick={() => setActiveTab('charts')}
        >
          Charts
        </button>
      </nav>

      <main className="app-main">
        {activeTab === 'overview' && renderOverviewTab()}
        {activeTab === 'control' && renderControlTab()}
        {activeTab === 'charts' && renderChartsTab()}
      </main>

      {/* Alert Messages */}
      {alertMessages.length > 0 && (
        <div className="alerts-container">
          {alertMessages.map(alert => (
            <div key={alert.id} className={`alert alert-${alert.type}`}>
              <span className="alert-time">{alert.timestamp}</span>
              <span className="alert-message">{alert.message}</span>
              <button 
                className="alert-close"
                onClick={() => setAlertMessages(prev => prev.filter(a => a.id !== alert.id))}
              >
                ×
              </button>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default App;