import {
  BatteryFull,
  CenterFocusStrong,
  ContentCopy,
  ChevronLeft,
  ChevronRight,
  HexagonOutlined,
  HomeOutlined,
  KeyboardArrowDown,
  KeyboardArrowLeft,
  KeyboardArrowRight,
  KeyboardArrowUp,
  MicNone,
  Refresh,
  SportsKabaddi,
  SettingsRounded,
  SmartToyOutlined,
  Speed,
  VideocamOutlined,
} from '@mui/icons-material';
import {
  alpha,
  Box,
  Button,
  Card,
  Checkbox,
  Chip,
  CssBaseline,
  Drawer,
  FormControlLabel,
  IconButton,
  LinearProgress,
  MenuItem,
  Paper,
  Slider,
  Stack,
  TextField,
  ThemeProvider,
  Typography,
} from '@mui/material';
import { useEffect, useMemo, useRef, useState, type ReactNode } from 'react';

import { VirtualJoystick } from './components/VirtualJoystick';
import { RobotApi } from './services/robotApi';
import { theme } from './theme';
import type { DashboardModel, RobotSettings, TransportMode, UiLanguage } from './types';


type TrackSelection = {
  x: number;
  y: number;
  w: number;
  h: number;
};

type SelectionPoint = {
  screenX: number;
  screenY: number;
  x: number;
  y: number;
};
type CameraPreviewUrls = {
  streamUrl: string;
  snapshotUrl: string;
};

const DEFAULT_ROBOT_NAME = 'WRobot-sdevil';
const copyByLanguage = {
  en: {
    gamepadMac: 'Gamepad MAC address',
    diagnostics: 'Diagnostics', liveDiagnostics: 'Live diagnostics', refreshDiagnostics: 'Refresh', copyDiagnostics: 'Copy', diagnosticsCopied: 'Copied', diagnosticsCopyFailed: 'Copy failed',
    online: 'Online', offline: 'Offline', idle: 'Idle', standby: 'Standby', active: 'Active', sitting: 'Sitting', noTarget: 'No Target', selectTarget: 'Tap a detected target to lock; tap again to unlock',
    moveControl: 'Drive Control', moveSpeed: 'Drive Speed', gimbalControl: 'Gimbal Control', gimbalSpeed: 'Gimbal Speed', legHeight: 'Leg Height', extendLegs: 'Extend legs', retractLegs: 'Retract legs', bodyLean: 'Body Lean', leanLeft: 'Lean left', leanRight: 'Lean right',
    robotStatus: 'Robot Status', oledPreview: 'OLED Face Preview', commonActions: 'Actions', settings: 'Settings', device: 'Device', network: 'Network', camera: 'Camera', maintenance: 'Advanced Maintenance',
    robot: 'Robot', detection: 'Target', attitude: 'Attitude', net: 'Network', battery: 'Battery', boardIp: 'Board IP', cameraStatus: 'Camera Status',
    stand: 'Stand', track: 'Track', sit: 'Sit', reset: 'Reset', jump: 'Jump', jumpForward: 'Jump Forward', jumpBackward: 'Jump Back', jumpLeft: 'Jump Left', jumpRight: 'Jump Right',
    language: 'Language', english: 'English', chinese: 'Chinese', robotName: 'Robot Name', controlMode: 'Control Mode', robotHost: 'Robot Host', homeWifiSsid: 'Home WiFi SSID', homeWifiPassword: 'Home WiFi Password', cameraStreamUrl: 'Camera Stream URL', cameraResolution: 'Camera Resolution', autoDetect: 'Auto detect',
    gamepadHint: 'Gamepad mode disables the web controller after reboot. Hold the IO34 button during power-on to force WiFi / Web recovery.',
    maintenanceOn: 'Maintenance mode enabled', maintenanceOff: 'Maintenance mode disabled', enterMaintenance: 'Enter Maintenance', exitMaintenance: 'Exit Maintenance', ledTest: 'LED Test', maintenanceHint: 'Maintenance mode stops motion control. Use it for LED tests, firmware updates, and firmware rollback.',
    saveApply: 'Save and Apply', ready: 'Ready', expression: 'Face', control: 'Control', ai: 'AI', voice: 'Voice', low: 'Low', medium: 'Medium', high: 'High', extreme: 'Extreme', saved: 'Saved',
    otaFirmware: 'Firmware Update', firmwareVersion: 'Firmware', cameraFirmwareVersion: 'Camera', selectFirmware: 'Select firmware .bin', otaFileExample: 'File name example: wrobot_firmware_3.2.51.bin', uploadFirmware: 'Upload Firmware', rollbackFirmware: 'Rollback Firmware', otaHint: 'Firmware OTA requires at least 30% battery, or manual confirmation of stable external power.', usbPowered: 'External power confirmed', passwordStored: 'Stored - leave blank to keep unchanged',
  },
  zh: {
    gamepadMac: '手柄 MAC 地址',
    diagnostics: '诊断', liveDiagnostics: '实时诊断', refreshDiagnostics: '刷新', copyDiagnostics: '复制', diagnosticsCopied: '已复制', diagnosticsCopyFailed: '复制失败',
    online: '在线', offline: '离线', idle: '空闲', standby: '待命', active: '运行中', sitting: '坐下', noTarget: '无目标', selectTarget: '点击识别目标锁定，再次点击解锁',
    moveControl: '移动控制', moveSpeed: '移动速度', gimbalControl: '云台控制', gimbalSpeed: '云台速度', legHeight: '腿部高度', extendLegs: '伸腿', retractLegs: '缩腿', bodyLean: '左右侧身', leanLeft: '向左侧身', leanRight: '向右侧身',
    robotStatus: '机器人状态', oledPreview: '表情屏预览', commonActions: '模式与动作', settings: '设置', device: '设备', network: '网络', camera: '相机', maintenance: '高级维护',
    robot: '机器人', detection: '识别', attitude: '姿态', net: '网络', battery: '电量', boardIp: '主板 IP', cameraStatus: '相机状态',
    stand: '站立', track: '追踪', sit: '坐下', reset: '复位', jump: '原地跳', jumpForward: '前跳', jumpBackward: '后跳', jumpLeft: '左跳', jumpRight: '右跳',
    language: '语言', english: 'English', chinese: '中文', robotName: '机器人名称', controlMode: '控制模式', robotHost: '机器人地址', homeWifiSsid: '家庭 WiFi SSID', homeWifiPassword: '家庭 WiFi 密码', cameraStreamUrl: '摄像头视频地址', cameraResolution: '摄像头分辨率', autoDetect: '自动检测',
    gamepadHint: '手柄模式重启后会关闭网页控制。开机时按住 IO34 按钮可强制进入 WiFi / Web 恢复模式。',
    maintenanceOn: '维护模式已开启', maintenanceOff: '维护模式已关闭', enterMaintenance: '进入维护', exitMaintenance: '退出维护', ledTest: '灯效测试', maintenanceHint: '维护模式会停止运动控制，用于灯效测试、固件升级和固件回滚。',
    saveApply: '保存并应用', ready: '就绪', expression: '表情', control: '控制', ai: 'AI 功能', voice: '语音', low: '低', medium: '中', high: '高', extreme: '极速', saved: '已保存',
    otaFirmware: '固件升级', firmwareVersion: '主板固件', cameraFirmwareVersion: '摄像头固件', selectFirmware: '选择固件 .bin', otaFileExample: '文件名示例：wrobot_firmware_3.2.51.bin', uploadFirmware: '上传固件', rollbackFirmware: '回滚固件', otaHint: '固件 OTA 升级需要电量至少 30%，或手动确认外部稳定供电。', usbPowered: '已确认外部供电', passwordStored: '已保存，留空则不修改',
  },
} as const;

function modeLabel(activeMode: string, copy: (typeof copyByLanguage)[UiLanguage]) {
  if (activeMode === 'track_mode') return copy.track;
  return copy.idle;
}


function targetStatusLabel(label: string, language: UiLanguage) {
  if (language === 'en') return label;
  const labels: Record<string, string> = {
    'No Target': '无目标',
    'Target Scan': '正在识别目标',
    'Target Visible': '发现目标',
    'Face Visible': '发现人脸',
    'Person Visible': '发现人物',
    'Track Classifying': '识别目标中',
    'Track Acquiring': '正在确认目标',
    'Target Locked': '目标已锁定',
    'Face Locked': '人脸已锁定',
    'Person Locked': '人物已锁定',
    'Ping Pong Ball': '乒乓球',
    'Target Occluded': '目标被遮挡',
    'Track Reacquiring': '正在重新捕获',
    'Target Lost': '目标已丢失',
    'Target Invalid': '目标无效',
    'Animal Bird': '动物：鸟',
    'Animal Cat': '动物：猫',
    'Animal Dog': '动物：狗',
    'Animal Horse': '动物：马',
    'Animal Sheep': '动物：羊',
    'Animal Cow': '动物：牛',
    'Animal Elephant': '动物：象',
    'Animal Bear': '动物：熊',
    'Animal Zebra': '动物：斑马',
    'Animal Giraffe': '动物：长颈鹿',
  };
  return labels[label] || label;
}

function cameraOverlayStatus(telemetry: DashboardModel, previewMode: 'stream' | 'snapshot' | 'none') {
  const label = telemetry.targetLabelRaw || '';
  if (label === 'TARGET_SCAN') return 'Scanning';
  if (label === 'TARGET_SELECTED') return 'Selecting';
  if (label === 'TARGET_INVALID') return 'No Match';
  if (label.endsWith('_LOCKED') || label === 'PING_PONG_BALL') return 'Locked';
  if (
    label.endsWith('_VISIBLE') ||
    label === 'FACE_VISIBLE' ||
    label === 'PERSON_VISIBLE' ||
    label.startsWith('ANIMAL_')
  ) {
    return 'Detected';
  }
  if (telemetry.cameraState === 'CONNECTED' || telemetry.cameraState === 'READY' || telemetry.cameraState === 'IDLE') {
    return previewMode === 'stream' ? 'Camera Live' : 'Camera Snapshot';
  }
  return telemetry.cameraState || 'Camera';
}

function isTargetLocked(rawLabel: string) {
  return /_LOCKED$|^LOCKED$/i.test(rawLabel || '');
}

function resolveInitialHost() {
  const currentHost = window.location.hostname;
  const savedHost = localStorage.getItem('wrobot.host') || '';
  if (
    currentHost &&
    currentHost !== 'localhost' &&
    currentHost !== '127.0.0.1'
  ) {
    return currentHost;
  }
  return savedHost || '192.168.8.1';
}

function normalizeGamepadMac(value: string) {
  const hex = value.replace(/[^0-9a-f]/gi, '').toLowerCase();
  if (!/^[0-9a-f]{12}$/.test(hex)) return '';
  return hex.match(/.{2}/g)?.join(':') || '';
}

function normalizeCameraUrls(rawUrl: string): CameraPreviewUrls {
  const trimmed = rawUrl.trim();
  if (!trimmed) {
    return { streamUrl: '', snapshotUrl: '' };
  }

  let base = trimmed;
  if (base.endsWith('/stream.mjpg')) {
    return {
      streamUrl: base,
      snapshotUrl: `${base.slice(0, -'/stream.mjpg'.length)}/snapshot.jpg`,
    };
  }
  if (base.endsWith('/snapshot.jpg')) {
    return {
      streamUrl: `${base.slice(0, -'/snapshot.jpg'.length)}/stream.mjpg`,
      snapshotUrl: base,
    };
  }
  if (base.endsWith('/')) {
    base = base.slice(0, -1);
  }
  return {
    streamUrl: `${base}/stream.mjpg`,
    snapshotUrl: `${base}/snapshot.jpg`,
  };
}

function withPreviewNonce(url: string) {
  if (!url) return '';
  const separator = url.includes('?') ? '&' : '?';
  return `${url}${separator}t=${Date.now()}`;
}

const initialHost = resolveInitialHost();
const initialCameraUrl = localStorage.getItem('wrobot.cameraUrl') || '';
const initialRobotName = DEFAULT_ROBOT_NAME;
const initialCameraResolution = '640x480';
const initialControlMode = 'wifi';
const initialTransport =
  (localStorage.getItem('wrobot.transport') as TransportMode | null) || 'auto';
const initialUiLanguage: UiLanguage = 'en';

const api = new RobotApi(initialHost, initialTransport);

type ActionLabelKey =
  | 'stand'
  | 'track'
  | 'sit'
  | 'reset'
  | 'jump'
  | 'jumpForward'
  | 'jumpBackward'
  | 'jumpLeft'
  | 'jumpRight';

const actionButtons: { labelKey: ActionLabelKey; action: string; icon: ReactNode }[] = [
  { labelKey: 'stand', action: 'stand', icon: <SmartToyOutlined /> },
  { labelKey: 'track', action: 'track_mode', icon: <CenterFocusStrong /> },
  { labelKey: 'sit', action: 'sit', icon: <MicNone /> },
  { labelKey: 'reset', action: 'reset', icon: <CenterFocusStrong /> },
  { labelKey: 'jump', action: 'jump', icon: <SportsKabaddi /> },
  { labelKey: 'jumpForward', action: 'jump_forward', icon: <SportsKabaddi /> },
  { labelKey: 'jumpBackward', action: 'jump_backward', icon: <SportsKabaddi /> },
  { labelKey: 'jumpLeft', action: 'jump_left', icon: <SportsKabaddi /> },
  { labelKey: 'jumpRight', action: 'jump_right', icon: <SportsKabaddi /> },
];

const initialTelemetry: DashboardModel = {
  bootId: 0,
  firmwareVersion: '',
  firmwareBuild: '',
  cameraFirmwareVersion: '',
  cameraFirmwareBuild: '',
  cameraStatusAgeMs: 0,
  cameraVersionStale: true,
  online: false,
  robotHost: initialHost,
  robotIp: initialHost,
  robotState: 'Offline',
  robotNetState: 'OFFLINE',
  batteryPercent: 0,
  batteryVoltage: 0,
  legHeightPercent: 50,
  legLeanPercent: 0,
  wifiDbm: 0,
  fps: 0,
  latencyMs: 0,
  aiMode: 'Standby',
  targetLabelRaw: 'NO_TARGET',
  targetLabel: 'No Target',
  speedMps: 0,
  pitch: 0,
  roll: 0,
  temperatureC: 0,
  cameraState: 'PENDING',
  cameraMessage: '',
  cameraResolution: '',
  cameraIp: '',
  cameraUrl: initialCameraUrl,
  clients: 0,
  otaRunning: false,
  otaProgress: 0,
  otaMessage: '',
  oledFace: '(^-^)',
  activeMode: '',
  maintenanceMode: false,
  controlTransport: 'Offline',
};

export default function App() {
  const [robotName, setRobotName] = useState(initialRobotName);
  const [uiLanguage, setUiLanguage] = useState<UiLanguage>(initialUiLanguage);
  const copy = copyByLanguage[uiLanguage];
  const [controlMode, setControlMode] = useState(initialControlMode);
  const [gamepadMac, setGamepadMac] = useState('');
  const [host, setHost] = useState(initialHost);
  const [hostDraft, setHostDraft] = useState(initialHost);
  const [cameraUrl, setCameraUrl] = useState(initialCameraUrl);
  const [cameraResolution, setCameraResolution] = useState(initialCameraResolution);
  const [cameraReloadNonce, setCameraReloadNonce] = useState(0);
  const previousCameraState = useRef(initialTelemetry.cameraState);
  const previousCameraResolution = useRef(initialTelemetry.cameraResolution);
  const [transport] = useState<TransportMode>(initialTransport);
  const [telemetry, setTelemetry] = useState<DashboardModel>(initialTelemetry);
  const [homeWifiSsid, setHomeWifiSsid] = useState('');
  const [savedHomeWifiSsid, setSavedHomeWifiSsid] = useState('');
  const [homeWifiPassword, setHomeWifiPassword] = useState('');
  const [settingsMessage, setSettingsMessage] = useState('');
  const [driveSpeed, setDriveSpeed] = useState(100);
  const [gimbalSpeed, setGimbalSpeed] = useState(100);
  const [legHeightDraft, setLegHeightDraft] = useState<number | null>(null);
  const [legLeanDraft, setLegLeanDraft] = useState(0);
  const [drawerOpen, setDrawerOpen] = useState(false);
  const [cameraPreviewActive, setCameraPreviewActive] = useState(false);
  const [wifiPasswordStored, setWifiPasswordStored] = useState(false);
  const [settingsBusy, setSettingsBusy] = useState(false);
  const [otaFile, setOtaFile] = useState<File | null>(null);
  const [otaProgress, setOtaProgress] = useState(0);
  const [otaBusy, setOtaBusy] = useState(false);
  const [usbPoweredOverride, setUsbPoweredOverride] = useState(false);
  const [diagnosticsLive, setDiagnosticsLive] = useState(false);
  const [diagnosticsText, setDiagnosticsText] = useState('');
  const [diagnosticsCopyStatus, setDiagnosticsCopyStatus] = useState('');
  const [driveControlActive, setDriveControlActive] = useState(false);
  const pollTimer = useRef<number | null>(null);
  const refreshSequence = useRef(0);
  const actionInFlight = useRef(false);
  const cameraUrlRef = useRef(cameraUrl);
  const cameraPreviewStickyTimer = useRef<number | null>(null);

  const effectiveCameraUrl = useMemo(
    () => telemetry.cameraUrl || cameraUrl,
    [cameraUrl, telemetry.cameraUrl],
  );
  const otaLocked = otaBusy || telemetry.otaRunning;
  const otaPowerOk = telemetry.batteryPercent >= 30 || usbPoweredOverride;
  const otaDisplayProgress = otaBusy
    ? otaProgress
    : telemetry.otaRunning
      ? telemetry.otaProgress
      : otaProgress;
  const otaDisplayMessage = otaBusy
    ? (settingsMessage || `Uploading ${otaProgress}%`)
    : telemetry.otaRunning
      ? (telemetry.otaMessage && telemetry.otaMessage !== 'Idle'
          ? telemetry.otaMessage
          : `${telemetry.otaProgress}%`)
      : telemetry.otaMessage || `${otaProgress}%`;

  function handleCameraPreviewActiveChange(active: boolean) {
    if (cameraPreviewStickyTimer.current !== null) {
      window.clearTimeout(cameraPreviewStickyTimer.current);
      cameraPreviewStickyTimer.current = null;
    }
    if (active) {
      setCameraPreviewActive(true);
      return;
    }
    cameraPreviewStickyTimer.current = window.setTimeout(() => {
      setCameraPreviewActive(false);
      cameraPreviewStickyTimer.current = null;
    }, 8000);
  }

  async function refresh(signal?: AbortSignal) {
    const sequence = ++refreshSequence.current;
    try {
      const next = await api.fetchStatus(cameraUrlRef.current, signal);
      if (sequence === refreshSequence.current) setTelemetry(next);
    } catch (error) {
      if (!(error instanceof DOMException && error.name === 'AbortError')) throw error;
    }
  }

  async function loadSettings(signal?: AbortSignal) {
    const settings = await api.fetchSettings(signal);
    if (!settings) return;
    applySettingsToUi(settings);
  }

  function applySettingsToUi(settings: RobotSettings) {
    setRobotName(settings.robot_name || DEFAULT_ROBOT_NAME);
    setControlMode(settings.control_mode || initialControlMode);
    setGamepadMac(settings.gamepad_mac || '');
    const nextLanguage = settings.ui_language === 'zh' ? 'zh' : 'en';
    setUiLanguage(nextLanguage);
    setHomeWifiSsid(settings.robot_wifi_ssid || '');
    setSavedHomeWifiSsid(settings.robot_wifi_ssid || '');
    setHomeWifiPassword('');
    setWifiPasswordStored(settings.robot_wifi_password_set === true);
    setCameraResolution(settings.camera_resolution || initialCameraResolution);
    if (settings.camera_url) {
      setCameraUrl(settings.camera_url);
      localStorage.setItem('wrobot.cameraUrl', settings.camera_url);
    }
  }

  function persistLocalSettings(nextHost = host) {
    localStorage.setItem('wrobot.host', nextHost);
    localStorage.setItem('wrobot.cameraUrl', cameraUrl);
    localStorage.setItem('wrobot.transport', transport);
  }

  async function handleSaveSettings() {
    if (settingsBusy || otaLocked) return;
    setSettingsBusy(true);
    setSettingsMessage('');
    const nextHost = hostDraft.trim() || host;
    if (nextHost !== host) {
      api.setHost(nextHost);
      setHost(nextHost);
    }
    persistLocalSettings(nextHost);
    const savedName = robotName.trim() || DEFAULT_ROBOT_NAME;
    setRobotName(savedName);
    const normalizedGamepadMac = normalizeGamepadMac(gamepadMac);
    if (controlMode === 'gamepad' && !normalizedGamepadMac) {
      setSettingsMessage('Invalid gamepad MAC address');
      setSettingsBusy(false);
      return;
    }
    if (normalizedGamepadMac) setGamepadMac(normalizedGamepadMac);
    const shouldApplyWifi =
      homeWifiSsid.trim() !== savedHomeWifiSsid.trim() ||
      homeWifiPassword.length > 0;
    try {
      const result = await api.saveAllSettings({
        robotName: savedName,
        uiLanguage,
        controlMode,
        gamepadMac: normalizedGamepadMac || gamepadMac,
        cameraUrl,
        cameraResolution,
        robotWifiSsid: homeWifiSsid,
        robotWifiPassword: homeWifiPassword,
        applyWifi: shouldApplyWifi,
      });
      setSettingsMessage(result.message || copy.saved);
      if (result.ok) {
        setHomeWifiPassword('');
        setWifiPasswordStored(wifiPasswordStored || Boolean(homeWifiPassword));
        await loadSettings();
        await refresh();
        setCameraReloadNonce((value) => value + 1);
        setDrawerOpen(false);
      }
    } finally {
      setSettingsBusy(false);
    }
  }

  async function refreshDiagnostics(signal?: AbortSignal) {
    const snapshot = await api.fetchDiagnostics(signal);
    if (snapshot) setDiagnosticsText(JSON.stringify(snapshot, null, 2));
  }

  async function copyDiagnostics() {
    if (!diagnosticsText) return;
    let copied = false;
    try {
      if (navigator.clipboard && window.isSecureContext) {
        await navigator.clipboard.writeText(diagnosticsText);
        copied = true;
      }
    } catch {
      copied = false;
    }

    if (!copied) {
      const textarea = document.createElement('textarea');
      textarea.value = diagnosticsText;
      textarea.setAttribute('readonly', '');
      textarea.style.position = 'fixed';
      textarea.style.left = '-9999px';
      textarea.style.top = '0';
      document.body.appendChild(textarea);
      textarea.focus();
      textarea.select();
      textarea.setSelectionRange(0, textarea.value.length);
      try {
        copied = document.execCommand('copy');
      } catch {
        copied = false;
      }
      document.body.removeChild(textarea);
    }

    setDiagnosticsCopyStatus(copied ? copy.diagnosticsCopied : copy.diagnosticsCopyFailed);
    window.setTimeout(() => setDiagnosticsCopyStatus(''), 1800);
  }

  async function handleActionPress(action: string) {
    if (actionInFlight.current || otaLocked) return;
    actionInFlight.current = true;
    const isTrackToggle = action === 'track_mode';
    const nextAction = isTrackToggle && telemetry.activeMode === action ? 'cancel_kick' : action;
    if (isTrackToggle) {
      setTelemetry((current) => ({
        ...current,
        activeMode: nextAction === 'track_mode' ? 'track_mode' : '',
      }));
    }
    try {
      const accepted = await api.sendAction(nextAction);
      if (!accepted && isTrackToggle) setSettingsMessage('Command rejected');
      if (accepted && isTrackToggle && nextAction === 'track_mode') {
        await api.sendTrackScan();
      }
      if (accepted) await new Promise((resolve) => window.setTimeout(resolve, 80));
      await refresh();
    } finally {
      actionInFlight.current = false;
    }
  }

  async function handleOtaUpload() {
    if (!otaFile || otaBusy) return;
    setOtaBusy(true);
    setOtaProgress(1);
    setSettingsMessage('Uploading firmware 1%');
    api.emergencyStop();
    api.disposeSocket();
    const result = await api.uploadFirmware(otaFile, (percent) => {
      setOtaProgress(percent);
      setSettingsMessage(`Uploading firmware ${percent}%`);
    }, usbPoweredOverride);
    setSettingsMessage(result.message);
    if (!result.ok && !result.connectionLost) {
      setOtaBusy(false);
      return;
    }
    if (result.ok) setOtaProgress(100);
    setSettingsMessage(result.connectionLost
      ? 'OTA response interrupted. Verifying the restarted firmware...'
      : 'Firmware uploaded. Waiting for robot restart...');
    const previousFirmwareVersion = telemetry.firmwareVersion;
    const restarted = await api.waitForRestart(telemetry.bootId, previousFirmwareVersion);
    if (restarted) {
      if (
        previousFirmwareVersion &&
        restarted.firmwareVersion &&
        restarted.firmwareVersion === previousFirmwareVersion
      ) {
        setTelemetry(restarted);
        setSettingsMessage('OTA restart detected, but firmware version did not change. Use USB flash and check the selected .bin file.');
        setOtaBusy(false);
        return;
      }
      setOtaFile(null);
      setTelemetry(restarted);
      setSettingsMessage('Firmware update complete');
      window.setTimeout(() => window.location.reload(), 800);
      return;
    } else {
      setSettingsMessage(result.connectionLost
        ? 'OTA could not be verified; the robot did not restart successfully'
        : 'Firmware uploaded, but robot did not reconnect in time');
    }
    setOtaBusy(false);
  }

  async function handleRollback() {
    if (otaLocked) return;
    setOtaBusy(true);
    api.emergencyStop();
    const result = await api.rollbackFirmware(usbPoweredOverride);
    setSettingsMessage(result.message);
    setOtaBusy(false);
  }

  useEffect(() => {
    cameraUrlRef.current = cameraUrl;
  }, [cameraUrl]);

  useEffect(() => {
    if (effectiveCameraUrl) return;
    if (cameraPreviewStickyTimer.current !== null) {
      window.clearTimeout(cameraPreviewStickyTimer.current);
      cameraPreviewStickyTimer.current = null;
    }
    setCameraPreviewActive(false);
  }, [effectiveCameraUrl]);

  useEffect(() => {
    if (!drawerOpen || !diagnosticsLive) return;
    const controller = new AbortController();
    void refreshDiagnostics(controller.signal);
    const timer = window.setInterval(() => {
      void refreshDiagnostics(controller.signal);
    }, 750);
    return () => {
      controller.abort();
      window.clearInterval(timer);
    };
  }, [diagnosticsLive, drawerOpen]);

  useEffect(() => {
    const stateBecameConnected =
      telemetry.cameraState === 'CONNECTED' &&
      previousCameraState.current !== 'CONNECTED';
    const activeResolutionChanged =
      Boolean(telemetry.cameraResolution) &&
      telemetry.cameraResolution !== previousCameraResolution.current;

    previousCameraState.current = telemetry.cameraState;
    previousCameraResolution.current = telemetry.cameraResolution;
    if (stateBecameConnected || activeResolutionChanged) {
      setCameraReloadNonce((value) => value + 1);
    }
  }, [telemetry.cameraResolution, telemetry.cameraState]);

  useEffect(() => {
    if (legHeightDraft === null) return;
    const timer = window.setTimeout(() => setLegHeightDraft(null), 900);
    return () => window.clearTimeout(timer);
  }, [legHeightDraft]);

  useEffect(() => {
    if (telemetry.maintenanceMode || otaLocked) return;
    setOtaFile(null);
    setOtaProgress(0);
    setUsbPoweredOverride(false);
  }, [telemetry.maintenanceMode, otaLocked]);

  useEffect(() => {
    api.setHost(host);
    api.setTransport(transport);
    const controller = new AbortController();
    let stopped = false;

    const poll = async () => {
      if (!driveControlActive && !otaLocked) await refresh(controller.signal);
      if (!stopped) pollTimer.current = window.setTimeout(poll, 1000);
    };
    void poll();
    if (!otaLocked) void loadSettings(controller.signal);

    const stopOnBackground = () => {
      if (document.hidden) api.emergencyStop();
    };
    document.addEventListener('visibilitychange', stopOnBackground);

    return () => {
      stopped = true;
      controller.abort();
      if (pollTimer.current) window.clearTimeout(pollTimer.current);
      document.removeEventListener('visibilitychange', stopOnBackground);
      api.disposeSocket();
    };
    // Initialization intentionally reruns only when the target host or transport changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [host, transport, driveControlActive, otaLocked]);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box sx={{ minHeight: '100vh', px: { xs: 1, sm: 1.5 }, py: 1 }}>
        <Box sx={{ maxWidth: 720, mx: 'auto' }}>
          <TopBar telemetry={telemetry} robotName={robotName.trim() || DEFAULT_ROBOT_NAME} copy={copy} />

          <Stack
            spacing={1.1}
            sx={{
              mt: 1,
              pointerEvents: otaLocked ? 'none' : 'auto',
              filter: otaLocked ? 'saturate(0.65)' : 'none',
            }}
          >
            <Box
              sx={{
                position: cameraPreviewActive ? 'sticky' : 'static',
                top: cameraPreviewActive ? 8 : 'auto',
                zIndex: cameraPreviewActive ? 12 : 'auto',
                pb: cameraPreviewActive ? 0.25 : 0,
                background: cameraPreviewActive
                  ? 'linear-gradient(180deg, rgba(7,10,16,0.96), rgba(7,10,16,0.82))'
                  : 'transparent',
              }}
            >
              <CameraCard
                key={`${effectiveCameraUrl || 'no-camera'}-${cameraReloadNonce}`}
                telemetry={telemetry}
                cameraUrl={otaLocked ? '' : effectiveCameraUrl}
                onPreviewActiveChange={handleCameraPreviewActiveChange}
                trackingEnabled={telemetry.activeMode === 'track_mode'}
                selectTargetLabel={copy.selectTarget}
                targetLocked={isTargetLocked(telemetry.targetLabelRaw)}
                onTrackSelection={(selection) =>
                  void api.sendTrackSelection(selection)
                }
                onTrackUnlock={() => void api.sendTrackUnlock()}
              />
            </Box>

            <MetricRow telemetry={telemetry} copy={copy} language={uiLanguage} />

            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: 'repeat(2, minmax(0, 1fr))',
                gap: 1,
                alignItems: 'stretch',
              }}
            >
              <ControlPanel>
                <VirtualJoystick
                  title={copy.moveControl}
                  footerLabel={copy.moveSpeed}
                  speedLabels={{ low: copy.low, medium: copy.medium, high: copy.high, extreme: copy.extreme }}
                  speedValue={driveSpeed}
                  onSpeedChange={setDriveSpeed}
                  onMove={(x, y) => {
                    setDriveControlActive(true);
                    void api.sendDrive(x, y, driveSpeed);
                  }}
                  onEnd={() => {
                    setDriveControlActive(false);
                    void api.sendDrive(0, 0, driveSpeed);
                  }}
                  beforeFooter={
                    <LeanSlider
                      label={copy.bodyLean}
                      value={legLeanDraft}
                      onChange={(value) => {
                        setLegLeanDraft(value);
                        api.sendLegLean(value);
                      }}
                      onRelease={() => {
                        setLegLeanDraft(0);
                        api.sendLegLean(0);
                      }}
                    />
                  }
                />
              </ControlPanel>

              <ControlPanel>
                <VirtualJoystick
                  title={copy.gimbalControl}
                  footerLabel={copy.gimbalSpeed}
                  speedLabels={{ low: copy.low, medium: copy.medium, high: copy.high, extreme: copy.extreme }}
                  speedValue={gimbalSpeed}
                  onSpeedChange={setGimbalSpeed}
                  onMove={(x, y) => {
                    void api.sendGimbal(x, y, gimbalSpeed);
                  }}
                  onEnd={() => {
                    void api.sendGimbal(0, 0, gimbalSpeed);
                  }}
                  beforeFooter={
                    <LegHeightSlider
                      label={copy.legHeight}
                      value={legHeightDraft ?? telemetry.legHeightPercent}
                      onChange={(value) => {
                        setLegHeightDraft(value);
                        api.sendLegHeightValue(value);
                      }}
                    />
                  }
                />
              </ControlPanel>
            </Box>

            <ModeActionStrip activeMode={telemetry.activeMode} copy={copy} onAction={handleActionPress} />

            <Box
              sx={{
                display: 'grid',
                gridTemplateColumns: 'repeat(2, minmax(0, 1fr))',
                gap: 1,
              }}
            >
              <InfoCard title={copy.robotStatus}>
                <StatusRows telemetry={telemetry} copy={copy} />
              </InfoCard>

              <InfoCard title={copy.oledPreview}>
                <OledPreview telemetry={telemetry} copy={copy} />
              </InfoCard>
            </Box>

            <BottomNav copy={copy} onOpenSettings={() => setDrawerOpen(true)} />
          </Stack>
        </Box>
      </Box>

      {otaLocked ? (
        <Box
          sx={{
            position: 'fixed',
            inset: 0,
            zIndex: 2000,
            display: 'grid',
            placeItems: 'center',
            bgcolor: 'rgba(2,6,14,.72)',
            backdropFilter: 'blur(6px)',
            px: 2,
          }}
        >
          <Paper
            sx={{
              width: 'min(360px, 92vw)',
              p: 2,
              borderRadius: 1.5,
              bgcolor: '#0d1017',
              border: '1px solid rgba(82,98,134,.35)',
            }}
          >
            <Stack spacing={1}>
              <Typography sx={{ fontSize: 15, fontWeight: 800 }}>
                {copy.otaFirmware}
              </Typography>
              <LinearProgress
                variant="determinate"
                value={otaDisplayProgress}
              />
              <Typography color="text.secondary" sx={{ fontSize: 12 }}>
                {otaDisplayMessage}
              </Typography>
            </Stack>
          </Paper>
        </Box>
      ) : null}

      <Drawer
        anchor="right"
        open={drawerOpen}
        onClose={() => setDrawerOpen(false)}
        slotProps={{ paper: { sx: { width: 'min(100%, 360px)', p: 2 } } }}
      >
        <Stack spacing={2}>
          <Stack
            direction="row"
            sx={{ justifyContent: 'space-between', alignItems: 'center' }}
          >
            <Typography variant="h6">{copy.settings}</Typography>
            <IconButton onClick={() => setDrawerOpen(false)}>
              <ChevronRight />
            </IconButton>
          </Stack>

          <Typography color="text.secondary" sx={{ fontSize: 12, fontWeight: 700 }}>{copy.device}</Typography>
          <TextField
            label={copy.robotName}
            size="small"
            value={robotName}
            onChange={(event) => setRobotName(event.target.value)}
          />
          <TextField
            select
            label={copy.language}
            size="small"
            value={uiLanguage}
            onChange={(event) => setUiLanguage(event.target.value as UiLanguage)}
          >
            <MenuItem value="en">English</MenuItem>
            <MenuItem value="zh">中文</MenuItem>
          </TextField>

          <TextField
            select
            label={copy.controlMode}
            size="small"
            value={controlMode}
            onChange={(event) => setControlMode(event.target.value)}
          >
            <MenuItem value="wifi">WiFi / Web</MenuItem>
            <MenuItem value="gamepad">Gamepad / Bluetooth</MenuItem>
          </TextField>

          {controlMode === 'gamepad' ? (
            <TextField
              label={copy.gamepadMac}
              size="small"
              value={gamepadMac}
              onChange={(event) => setGamepadMac(event.target.value)}
              slotProps={{ htmlInput: { inputMode: 'text', autoCapitalize: 'none', spellCheck: false } }}
            />
          ) : null}

          <TextField
            label={copy.robotHost}
            size="small"
            value={hostDraft}
            onChange={(event) => setHostDraft(event.target.value)}
          />
          <Typography color="text.secondary" sx={{ fontSize: 11, lineHeight: 1.45 }}>
            {copy.gamepadHint}
          </Typography>

          <Typography color="text.secondary" sx={{ fontSize: 12, fontWeight: 700 }}>{copy.network}</Typography>
          <TextField
            label={copy.homeWifiSsid}
            size="small"
            value={homeWifiSsid}
            onChange={(event) => setHomeWifiSsid(event.target.value)}
          />
          <TextField
            label={copy.homeWifiPassword}
            size="small"
            type="password"
            placeholder={wifiPasswordStored ? '********' : undefined}
            slotProps={{ inputLabel: { shrink: wifiPasswordStored || Boolean(homeWifiPassword) } }}
            value={homeWifiPassword}
            onChange={(event) => setHomeWifiPassword(event.target.value)}
            helperText={wifiPasswordStored && !homeWifiPassword ? copy.passwordStored : undefined}
          />

          <Typography color="text.secondary" sx={{ fontSize: 12, fontWeight: 700 }}>{copy.camera}</Typography>
          <TextField
            label={copy.cameraStreamUrl}
            size="small"
            placeholder={copy.autoDetect}
            value={cameraUrl}
            onChange={(event) => setCameraUrl(event.target.value)}
          />
          <TextField
            select
            label={copy.cameraResolution}
            size="small"
            value={cameraResolution}
            onChange={(event) => setCameraResolution(event.target.value)}
          >
            <MenuItem value="160x120">160 x 120 (Remote control)</MenuItem>
            <MenuItem value="320x240">320 x 240 (Low latency)</MenuItem>
            <MenuItem value="640x480">640 x 480</MenuItem>
            <MenuItem value="1280x720">1280 x 720</MenuItem>
          </TextField>

          <SmallActionButton disabled={settingsBusy || otaLocked} onClick={handleSaveSettings}>{copy.saveApply}</SmallActionButton>

          <Chip
            label={settingsMessage || copy.ready}
            sx={{
              alignSelf: 'flex-start',
              bgcolor: alpha('#122033', 0.92),
              color: '#9bbdff',
              borderRadius: 2,
            }}
          />

          <Stack
            spacing={1.15}
            sx={{
              p: 1.25,
              borderRadius: 1.5,
              border: `1px solid ${telemetry.maintenanceMode ? alpha('#69e78b', 0.52) : alpha('#7895c5', 0.25)}`,
              bgcolor: telemetry.maintenanceMode ? alpha('#123621', 0.24) : alpha('#080d17', 0.48),
              transition: 'border-color 160ms ease, background-color 160ms ease',
            }}
          >
            <Stack direction="row" spacing={1} sx={{ alignItems: 'center', justifyContent: 'space-between' }}>
              <Typography color="text.primary" sx={{ fontSize: 12, fontWeight: 800 }}>
                {copy.maintenance}
              </Typography>
              <Chip
                label={telemetry.maintenanceMode ? copy.maintenanceOn : copy.maintenanceOff}
                sx={{
                  bgcolor: telemetry.maintenanceMode ? alpha('#1f5f3b', 0.5) : alpha('#122033', 0.92),
                  color: telemetry.maintenanceMode ? '#88f5a3' : '#9bbdff',
                  borderRadius: 1.25,
                  height: 24,
                  fontSize: 10,
                }}
              />
            </Stack>
            <Stack direction="row" spacing={1}>
              <SmallActionButton disabled={telemetry.maintenanceMode || otaLocked} onClick={() => void api.sendAction('maintenance_on')}>{copy.enterMaintenance}</SmallActionButton>
              <SmallActionButton disabled={!telemetry.maintenanceMode || otaLocked} onClick={() => void api.sendAction('maintenance_off')}>{copy.exitMaintenance}</SmallActionButton>
            </Stack>
            <SmallActionButton disabled={!telemetry.maintenanceMode || otaLocked} onClick={() => void api.sendAction('led_test')}>{copy.ledTest}</SmallActionButton>
            <Typography color="text.secondary" sx={{ fontSize: 11, lineHeight: 1.45 }}>
              {copy.maintenanceHint}
            </Typography>

            <Box sx={{ height: '1px', bgcolor: alpha('#8ba8d8', 0.18) }} />
            <Stack direction="row" sx={{ alignItems: 'center', justifyContent: 'space-between', gap: 1 }}>
              <Typography color="text.secondary" sx={{ fontSize: 12, fontWeight: 700 }}>
                {copy.diagnostics}
              </Typography>
              <FormControlLabel
                control={
                  <Checkbox
                    size="small"
                    checked={diagnosticsLive}
                    onChange={(event) => setDiagnosticsLive(event.target.checked)}
                  />
                }
                label={copy.liveDiagnostics}
                sx={{ m: 0, '& .MuiFormControlLabel-label': { fontSize: 11 } }}
              />
            </Stack>
            <Stack direction="row" spacing={1}>
              <Button
                size="small"
                variant="outlined"
                startIcon={<Refresh />}
                onClick={() => void refreshDiagnostics()}
              >
                {copy.refreshDiagnostics}
              </Button>
              <Button
                size="small"
                variant="outlined"
                startIcon={<ContentCopy />}
                disabled={!diagnosticsText}
                onClick={() => void copyDiagnostics()}
              >
                {copy.copyDiagnostics}
              </Button>
            </Stack>
            {diagnosticsCopyStatus && (
              <Typography color={diagnosticsCopyStatus === copy.diagnosticsCopied ? 'success.main' : 'error.main'} sx={{ fontSize: 11 }}>
                {diagnosticsCopyStatus}
              </Typography>
            )}
            <Box
              component="pre"
              sx={{
                m: 0,
                p: 1,
                maxHeight: 260,
                overflow: 'auto',
                borderRadius: 1,
                bgcolor: '#060910',
                border: '1px solid rgba(82,98,134,.22)',
                color: '#9fc0ff',
                fontFamily: 'monospace',
                fontSize: 10,
                lineHeight: 1.45,
                whiteSpace: 'pre-wrap',
                overflowWrap: 'anywhere',
              }}
            >
              {diagnosticsText || '{}'}
            </Box>

            <Box sx={{ height: '1px', bgcolor: alpha('#8ba8d8', 0.18) }} />
            <Typography color="text.secondary" sx={{ fontSize: 12, fontWeight: 700 }}>
              {copy.otaFirmware}
            </Typography>
            <Button
              component="label"
              variant="outlined"
              size="small"
              disabled={!telemetry.maintenanceMode || otaLocked}
            >
              {otaFile?.name || copy.selectFirmware}
              <input
                hidden
                type="file"
                disabled={!telemetry.maintenanceMode || otaLocked}
                accept=".bin,application/octet-stream"
                onChange={(event) => {
                  setOtaFile(event.target.files?.[0] || null);
                  setOtaProgress(0);
                }}
              />
            </Button>
            <Typography color="text.secondary" sx={{ fontSize: 10.5, lineHeight: 1.35 }}>
              {copy.otaFileExample}
            </Typography>
            {(otaBusy || telemetry.otaRunning || otaProgress > 0) ? (
              <Stack spacing={0.5}>
                <LinearProgress
                  variant="determinate"
                  value={otaDisplayProgress}
                />
                <Typography color="text.secondary" sx={{ fontSize: 11 }}>
                  {otaDisplayMessage}
                </Typography>
              </Stack>
            ) : null}
            <FormControlLabel
              control={
                <Checkbox
                  size="small"
                  checked={usbPoweredOverride}
                  disabled={!telemetry.maintenanceMode || otaLocked}
                  onChange={(event) => setUsbPoweredOverride(event.target.checked)}
                />
              }
              label={copy.usbPowered}
              sx={{
                m: 0,
                color: 'text.secondary',
                '& .MuiFormControlLabel-label': { fontSize: 12, fontWeight: 700 },
              }}
            />
            <Stack direction="row" spacing={1}>
              <SmallActionButton
                disabled={!telemetry.maintenanceMode || !otaPowerOk || !otaFile || otaLocked}
                onClick={() => void handleOtaUpload()}
              >
                {copy.uploadFirmware}
              </SmallActionButton>
              <SmallActionButton
                disabled={!telemetry.maintenanceMode || !otaPowerOk || otaLocked}
                onClick={() => void handleRollback()}
              >
                {copy.rollbackFirmware}
              </SmallActionButton>
            </Stack>
            <Typography color="text.secondary" sx={{ fontSize: 11, lineHeight: 1.45 }}>
              {copy.otaHint}
            </Typography>
          </Stack>

          <Stack
            spacing={0.25}
            sx={{
              mt: 1,
              pt: 1.5,
              borderTop: `1px solid ${alpha('#8ba8d8', 0.18)}`,
            }}
          >
            <Typography color="text.secondary" sx={{ fontSize: 11, fontWeight: 700 }}>
              {copy.firmwareVersion} {telemetry.firmwareVersion || 'unknown'}
            </Typography>
            <Typography color="text.secondary" sx={{ mt: 0.5, fontSize: 11, fontWeight: 700 }}>
              {copy.cameraFirmwareVersion}{' '}
              {telemetry.cameraVersionStale
                ? telemetry.cameraFirmwareVersion
                  ? `cached ${telemetry.cameraFirmwareVersion}`
                  : 'unknown'
                : telemetry.cameraFirmwareVersion || 'unknown'}
            </Typography>
          </Stack>
        </Stack>
      </Drawer>
    </ThemeProvider>
  );
}

function TopBar({
  telemetry,
  robotName,
  copy,
}: {
  telemetry: DashboardModel;
  robotName: string;
  copy: (typeof copyByLanguage)[UiLanguage];
}) {
  return (
    <Stack
      direction="row"
      sx={{
        alignItems: 'center',
        gap: 1,
      }}
    >
      <Paper
        sx={{
          width: 42,
          height: 42,
          borderRadius: 2.25,
          bgcolor: '#0f1320',
          display: 'grid',
          placeItems: 'center',
          flexShrink: 0,
        }}
      >
        <SmartToyOutlined sx={{ color: '#73a5ff', fontSize: 20 }} />
      </Paper>

      <Box sx={{ minWidth: 0, flex: 1 }}>
        <Stack direction="row" spacing={0.8} sx={{ alignItems: 'center' }}>
          <Typography sx={{ fontSize: 18, fontWeight: 700, lineHeight: 1.05 }}>
            {robotName}
          </Typography>
          <Box
            sx={{
              width: 8,
              height: 8,
              borderRadius: '50%',
              bgcolor: telemetry.online ? 'success.main' : 'error.main',
            }}
          />
          <Typography sx={{ color: '#78e5a1', fontSize: 11, fontWeight: 600 }}>
            {telemetry.online ? copy.online : copy.offline}
          </Typography>
        </Stack>
        <Typography color="text.secondary" sx={{ fontSize: 11 }}>
          {telemetry.robotIp}
        </Typography>
      </Box>

      <TopBatteryPill telemetry={telemetry} />
    </Stack>
  );
}

function TopBatteryPill({ telemetry }: { telemetry: DashboardModel }) {
  return (
    <Box
      sx={{
        px: 0.25,
        py: 0.25,
        display: 'flex',
        alignItems: 'center',
        gap: 0.45,
        flexShrink: 0,
      }}
    >
      <BatteryFull sx={{ color: '#86f0a0', fontSize: 16 }} />
      <Typography sx={{ fontWeight: 700, fontSize: 11, lineHeight: 1 }}>
        {telemetry.online
          ? `${telemetry.batteryPercent}% (${telemetry.batteryVoltage.toFixed(1)}V)`
          : '--% (--.-V)'}
      </Typography>
    </Box>
  );
}

function CameraCard({
  telemetry,
  cameraUrl,
  onPreviewActiveChange,
  trackingEnabled,
  selectTargetLabel,
  targetLocked,
  onTrackSelection,
  onTrackUnlock,
}: {
  telemetry: DashboardModel;
  cameraUrl: string;
  onPreviewActiveChange: (active: boolean) => void;
  trackingEnabled: boolean;
  selectTargetLabel: string;
  targetLocked: boolean;
  onTrackSelection: (selection: TrackSelection) => void;
  onTrackUnlock: () => void;
}) {
  const { streamUrl, snapshotUrl } = useMemo(
    () => normalizeCameraUrls(cameraUrl),
    [cameraUrl],
  );
  const [previewMode, setPreviewMode] = useState<'stream' | 'snapshot' | 'none'>(
    streamUrl ? 'stream' : snapshotUrl ? 'snapshot' : 'none',
  );
  const [previewSrc, setPreviewSrc] = useState(withPreviewNonce(streamUrl || snapshotUrl));
  const [previewError, setPreviewError] = useState('');
  const lastPreviewLoadMs = useRef(0);
  const [selectionStart, setSelectionStart] = useState<SelectionPoint | null>(null);
  const [selectionEnd, setSelectionEnd] = useState<SelectionPoint | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);

  useEffect(
    () => () => {
      onPreviewActiveChange(false);
    },
    [onPreviewActiveChange],
  );

  useEffect(() => {
    if (streamUrl) {
      setPreviewMode('stream');
      setPreviewSrc(withPreviewNonce(streamUrl));
      setPreviewError('');
      return;
    }
    if (snapshotUrl) {
      setPreviewMode('snapshot');
      setPreviewSrc(withPreviewNonce(snapshotUrl));
      setPreviewError('');
      return;
    }
    setPreviewMode('none');
    setPreviewSrc('');
    setPreviewError('No Video Signal');
    onPreviewActiveChange(false);
  }, [onPreviewActiveChange, snapshotUrl, streamUrl]);

  useEffect(() => {
    if (previewMode !== 'snapshot' || !snapshotUrl) return;
    const refreshSnapshot = () => {
      if (!document.hidden) setPreviewSrc(withPreviewNonce(snapshotUrl));
    };
    const timer = window.setInterval(refreshSnapshot, 500);
    const retry = streamUrl
      ? window.setTimeout(() => {
          setPreviewMode('stream');
          setPreviewSrc(withPreviewNonce(streamUrl));
        }, 5000)
      : null;
    return () => {
      window.clearInterval(timer);
      if (retry !== null) window.clearTimeout(retry);
    };
  }, [previewMode, snapshotUrl, streamUrl]);

  useEffect(() => {
    if (previewMode !== 'none' || !streamUrl) return;
    const retry = window.setTimeout(() => {
      setPreviewError('');
      setPreviewMode('stream');
      setPreviewSrc(withPreviewNonce(streamUrl));
    }, 5000);
    return () => window.clearTimeout(retry);
  }, [previewMode, streamUrl]);

  useEffect(() => {
    if (previewMode !== 'stream' || !streamUrl) return;
    const reconnect = () => {
      if (document.hidden) return;
      const image = imageRef.current;
      const loaded = image && image.complete && image.naturalWidth > 0 && image.naturalHeight > 0;
      const stale = lastPreviewLoadMs.current > 0 && Date.now() - lastPreviewLoadMs.current > 20000;
      if (!loaded || stale) {
        setPreviewError('');
        setPreviewSrc(withPreviewNonce(streamUrl));
      }
    };
    const timer = window.setInterval(reconnect, 7000);
    window.addEventListener('focus', reconnect);
    document.addEventListener('visibilitychange', reconnect);
    return () => {
      window.clearInterval(timer);
      window.removeEventListener('focus', reconnect);
      document.removeEventListener('visibilitychange', reconnect);
    };
  }, [previewMode, streamUrl]);

  function pointerToSelectionPoint(clientX: number, clientY: number) {
    const container = containerRef.current;
    const previewImage = imageRef.current;
    if (!container || !previewImage || !previewImage.naturalWidth || !previewImage.naturalHeight) {
      return null;
    }

    const bounds = container.getBoundingClientRect();
    const scale = Math.min(
      bounds.width / previewImage.naturalWidth,
      bounds.height / previewImage.naturalHeight,
    );
    const renderedWidth = previewImage.naturalWidth * scale;
    const renderedHeight = previewImage.naturalHeight * scale;
    const offsetX = (bounds.width - renderedWidth) / 2;
    const offsetY = (bounds.height - renderedHeight) / 2;
    const imageX = Math.max(0, Math.min(renderedWidth, clientX - bounds.left - offsetX));
    const imageY = Math.max(0, Math.min(renderedHeight, clientY - bounds.top - offsetY));

    return {
      screenX: offsetX + imageX,
      screenY: offsetY + imageY,
      x: Math.round((imageX / renderedWidth) * 10000),
      y: Math.round((imageY / renderedHeight) * 10000),
    };
  }

  function finishSelection(endPoint: SelectionPoint | null) {
    if (!selectionStart || !endPoint) {
      setSelectionStart(null);
      setSelectionEnd(null);
      return;
    }
    if (targetLocked) {
      onTrackUnlock();
      setSelectionStart(null);
      setSelectionEnd(null);
      return;
    }

    const screenWidth = Math.abs(endPoint.screenX - selectionStart.screenX);
    const screenHeight = Math.abs(endPoint.screenY - selectionStart.screenY);
    let x = Math.min(selectionStart.x, endPoint.x);
    let y = Math.min(selectionStart.y, endPoint.y);
    let w = Math.abs(endPoint.x - selectionStart.x);
    let h = Math.abs(endPoint.y - selectionStart.y);
    if (screenWidth < 16 || screenHeight < 16) {
      const centerX = endPoint.x;
      const centerY = endPoint.y;
      w = 2800;
      h = 2800;
      x = Math.max(0, Math.min(10000 - w, centerX - Math.round(w / 2)));
      y = Math.max(0, Math.min(10000 - h, centerY - Math.round(h / 2)));
    } else {
      const padX = 800;
      const padY = 800;
      x = Math.max(0, x - padX);
      y = Math.max(0, y - padY);
      w = Math.min(10000 - x, w + padX * 2);
      h = Math.min(10000 - y, h + padY * 2);
    }
    onTrackSelection({
      x,
      y,
      w: Math.max(1, w),
      h: Math.max(1, h),
    });
    setSelectionStart(null);
    setSelectionEnd(null);
  }

  const selectionRect =
    trackingEnabled && selectionStart && selectionEnd
      ? {
          left: Math.min(selectionStart.screenX, selectionEnd.screenX),
          top: Math.min(selectionStart.screenY, selectionEnd.screenY),
          width: Math.abs(selectionEnd.screenX - selectionStart.screenX),
          height: Math.abs(selectionEnd.screenY - selectionStart.screenY),
        }
      : null;

  return (
    <Card sx={{ p: 0, overflow: 'hidden' }}>
      <Box
        ref={containerRef}
        onPointerDown={(event) => {
          if (!trackingEnabled || previewMode !== 'stream' || !previewSrc) return;
          const point = pointerToSelectionPoint(event.clientX, event.clientY);
          if (!point) return;
          event.currentTarget.setPointerCapture(event.pointerId);
          setSelectionStart(point);
          setSelectionEnd(point);
        }}
        onPointerMove={(event) => {
          if (!trackingEnabled || !selectionStart || !event.currentTarget.hasPointerCapture(event.pointerId)) return;
          const point = pointerToSelectionPoint(event.clientX, event.clientY);
          if (point) setSelectionEnd(point);
        }}
        onPointerUp={(event) => {
          if (!trackingEnabled || !selectionStart) return;
          const point = pointerToSelectionPoint(event.clientX, event.clientY);
          if (point) setSelectionEnd(point);
          event.currentTarget.releasePointerCapture(event.pointerId);
          finishSelection(point);
        }}
        onPointerCancel={() => {
          if (selectionStart && selectionEnd) finishSelection(selectionEnd);
          else {
            setSelectionStart(null);
            setSelectionEnd(null);
          }
        }}
        sx={{
          position: 'relative',
          aspectRatio: '16 / 10',
          overflow: 'hidden',
          bgcolor: '#090c12',
          touchAction: trackingEnabled ? 'none' : 'auto',
          cursor: trackingEnabled ? 'crosshair' : 'default',
        }}
      >
        {previewSrc ? (
          <Box
            ref={imageRef}
            component="img"
            src={previewSrc}
            alt="camera preview"
            draggable={false}
            onError={() => {
              if (previewMode === 'stream' && snapshotUrl) {
                setPreviewMode('snapshot');
                setPreviewSrc(withPreviewNonce(snapshotUrl));
                setPreviewError('');
              } else {
                setPreviewMode('none');
                setPreviewSrc('');
                setPreviewError('No Video Signal');
              }
              onPreviewActiveChange(false);
            }}
            onLoad={() => {
              lastPreviewLoadMs.current = Date.now();
              setPreviewError('');
              onPreviewActiveChange(previewMode === 'stream');
            }}
            sx={{
              width: '100%',
              height: '100%',
              objectFit: 'contain',
              imageRendering: 'auto',
              display: 'block',
              filter: 'contrast(1.08) saturate(1.04) brightness(1.01)',
              userSelect: 'none',
              pointerEvents: 'none',
            }}
          />
        ) : (
          <Box
            sx={{
              position: 'absolute',
              inset: 0,
              display: 'grid',
              placeItems: 'center',
              color: '#98a7bd',
              fontSize: 13,
              fontWeight: 600,
              textAlign: 'center',
              px: 2,
            }}
          >
            {previewError || 'No Video Signal'}
          </Box>
        )}

        {selectionRect ? (
          <Box
            sx={{
              position: 'absolute',
              left: selectionRect.left,
              top: selectionRect.top,
              width: selectionRect.width,
              height: selectionRect.height,
              border: '2px solid #4f8cff',
              bgcolor: alpha('#3478ff', 0.12),
              pointerEvents: 'none',
            }}
          />
        ) : null}

        {trackingEnabled && previewMode === 'stream' && previewSrc ? (
          <Box
            sx={{
              position: 'absolute',
              left: '50%',
              bottom: 10,
              transform: 'translateX(-50%)',
              px: 1,
              py: 0.45,
              borderRadius: 1,
              bgcolor: alpha('#080b12', 0.76),
              color: '#ffffff',
              fontSize: 11,
              fontWeight: 700,
              whiteSpace: 'nowrap',
              pointerEvents: 'none',
            }}
          >
            {selectTargetLabel}
          </Box>
        ) : null}

        {previewSrc ? (
          <Chip
            label={cameraOverlayStatus(telemetry, previewMode)}
            sx={{
              position: 'absolute',
              top: 10,
              left: 10,
              color: '#84f8b0',
              fontWeight: 700,
              fontSize: 11,
              height: 24,
              bgcolor: alpha('#153225', 0.54),
              pointerEvents: 'none',
            }}
          />
        ) : null}
      </Box>
    </Card>
  );
}
function MetricRow({
  telemetry,
  copy,
  language,
}: {
  telemetry: DashboardModel;
  copy: (typeof copyByLanguage)[UiLanguage];
  language: UiLanguage;
}) {
  const items = [
    {
      icon: <SmartToyOutlined sx={{ color: '#85f0a1', fontSize: 18 }} />,
      label: copy.robot,
      value: telemetry.activeMode ? modeLabel(telemetry.activeMode, copy) : telemetry.robotState,
    },
    {
      icon: <CenterFocusStrong sx={{ color: '#79a9ff', fontSize: 18 }} />,
      label: copy.detection,
      value: targetStatusLabel(telemetry.targetLabel, language),
    },
    {
      icon: <Speed sx={{ color: '#a98bff', fontSize: 18 }} />,
      label: copy.attitude,
      value: `${telemetry.pitch.toFixed(1)} deg`,
    },
    {
      icon: <HomeOutlined sx={{ color: '#7db2ff', fontSize: 18 }} />,
      label: copy.net,
      value: `${telemetry.robotNetState} / ${telemetry.controlTransport}`,
    },
    {
      icon: <VideocamOutlined sx={{ color: '#ffb067', fontSize: 18 }} />,
      label: copy.camera,
      value: telemetry.cameraResolution
        ? `${telemetry.cameraState} / ${telemetry.cameraResolution}`
        : telemetry.cameraState,
    },

    ...(telemetry.otaRunning
      ? [
          {
            icon: <HexagonOutlined sx={{ color: '#ff8a76', fontSize: 18 }} />,
            label: 'OTA',
            value: `${telemetry.otaProgress}%`,
          },
        ]
      : []),
  ];

  return (
    <PagedStrip columns="calc((100% - 16px) / 3)" pageSize={3} itemCount={items.length}>
      {items.map((item) => (
        <Paper
          key={item.label}
          sx={{
            minWidth: 'calc((100% - 16px) / 3)',
            minHeight: 54,
            px: 0.9,
            py: 0.72,
            borderRadius: 1.25,
            display: 'flex',
            alignItems: 'center',
            gap: 0.9,
            bgcolor: '#0d1017',
            border: '1px solid rgba(82,98,134,.28)',
            boxSizing: 'border-box',
          }}
        >
          {item.icon}
          <Box sx={{ minWidth: 0 }}>
            <Typography
              variant="caption"
              color="text.secondary"
              sx={{ display: 'block', lineHeight: 1.05, fontSize: 9.5 }}
            >
              {item.label}
            </Typography>
            <Typography
              sx={{
                fontSize: 11.5,
                fontWeight: 700,
                whiteSpace: 'nowrap',
                overflow: 'hidden',
                textOverflow: 'ellipsis',
              }}
            >
              {item.value}
            </Typography>
          </Box>
        </Paper>
      ))}
    </PagedStrip>
  );
}

function ControlPanel({ children }: { children: ReactNode }) {
  return (
    <Card sx={{ height: '100%' }}>
      <Box sx={{ p: 1.2, height: '100%' }}>{children}</Box>
    </Card>
  );
}

function ModeActionStrip({
  activeMode,
  copy,
  onAction,
}: {
  activeMode: string;
  copy: (typeof copyByLanguage)[UiLanguage];
  onAction: (action: string) => void | Promise<void>;
}) {
  const toggleModes = new Set(['track_mode']);

  return (
    <InfoCard title={copy.commonActions}>
      <PagedStrip columns="calc((100% - 24px) / 4)" pageSize={4} itemCount={actionButtons.length}>
        {actionButtons.map((item) => {
          const selected = toggleModes.has(item.action) && activeMode === item.action;
          return (
            <Button
              key={item.action}
              variant={selected ? 'contained' : 'outlined'}
              startIcon={item.icon}
              sx={{
                minWidth: 'calc((100% - 24px) / 4)',
                minHeight: 68,
                flexDirection: 'column',
                gap: 0.35,
                borderRadius: 1.25,
                borderColor: selected ? '#3a76ff' : '#263041',
                fontSize: 10.5,
                lineHeight: 1.1,
                px: 0.35,
                py: 0.6,
                color: selected ? '#ffffff' : undefined,
                bgcolor: selected ? undefined : '#0d1017',
                background: selected ? 'linear-gradient(180deg,#2d6dff,#184fcc)' : undefined,
                '& .MuiButton-startIcon': {
                  margin: 0,
                  '& svg': {
                    fontSize: 18,
                  },
                },
              }}
              onClick={() => {
                void onAction(item.action);
              }}
            >
              {copy[item.labelKey]}
            </Button>
          );
        })}
      </PagedStrip>
    </InfoCard>
  );
}

function InfoCard({
  title,
  children,
}: {
  title: string;
  children: ReactNode;
}) {
  return (
    <Card>
      <Box sx={{ p: 1.3 }}>
        <Typography sx={{ fontSize: 15, fontWeight: 700, mb: 1 }}>
          {title}
        </Typography>
        {children}
      </Box>
    </Card>
  );
}

function StatusRows({ telemetry, copy }: { telemetry: DashboardModel; copy: (typeof copyByLanguage)[UiLanguage] }) {
  const rows = [
    {
      icon: <BatteryFull sx={{ color: '#85f0a1', fontSize: 18 }} />,
      label: copy.battery,
      value: `${telemetry.batteryPercent}% (${telemetry.batteryVoltage.toFixed(1)}V)`,
    },
    {
      icon: <HomeOutlined sx={{ color: '#7db2ff', fontSize: 18 }} />,
      label: copy.boardIp,
      value: telemetry.robotIp,
    },
    {
      icon: <VideocamOutlined sx={{ color: '#ffb067', fontSize: 18 }} />,
      label: copy.cameraStatus,
      value: telemetry.cameraResolution
        ? `${telemetry.cameraState} / ${telemetry.cameraResolution}`
        : telemetry.cameraState,
    },
  ];

  return (
    <Stack spacing={0.6}>
      {rows.map((row) => (
        <Stack
          key={row.label}
          direction="row"
          sx={{
            minHeight: 42,
            alignItems: 'center',
            justifyContent: 'space-between',
            gap: 1,
            px: 0.85,
            py: 0.72,
            borderRadius: 1.5,
            bgcolor: '#0d1017',
            border: '1px solid rgba(82,98,134,.16)',
          }}
        >
          <Stack direction="row" spacing={1} sx={{ alignItems: 'center', minWidth: 0 }}>
            {row.icon}
            <Typography color="text.secondary" sx={{ fontSize: 12 }}>
              {row.label}
            </Typography>
          </Stack>
          <Typography
            sx={{
              color: '#74e49d',
              fontWeight: 700,
              fontSize: 12,
              whiteSpace: 'nowrap',
              overflow: 'hidden',
              textOverflow: 'ellipsis',
            }}
          >
            {row.value}
          </Typography>
        </Stack>
      ))}
    </Stack>
  );
}

function OledPreview({ telemetry, copy }: { telemetry: DashboardModel; copy: (typeof copyByLanguage)[UiLanguage] }) {
  return (
    <Stack spacing={0.95}>
      <Box
        sx={{
          borderRadius: 1.5,
          bgcolor: '#000',
          minHeight: 108,
          aspectRatio: '16 / 9',
          display: 'grid',
          placeItems: 'center',
          fontFamily: 'Consolas, monospace',
          fontSize: 22,
          fontWeight: 700,
          color: '#3f79ff',
          letterSpacing: 2,
        }}
      >
        {telemetry.oledFace}
      </Box>

      <Stack direction="row" spacing={1} sx={{ alignItems: 'center' }}>
        <IconButton sx={miniRoundButtonSx}>
          <ChevronLeft />
        </IconButton>
        <Button
          fullWidth
          variant="outlined"
          startIcon={<SettingsRounded />}
          sx={{
            minHeight: 38,
            borderRadius: 1.75,
            fontSize: 12,
            px: 0.75,
            whiteSpace: 'nowrap',
          }}
        >{copy.expression}</Button>
        <IconButton sx={miniRoundButtonSx}>
          <ChevronRight />
        </IconButton>
      </Stack>
    </Stack>
  );
}

function BottomNav({ copy, onOpenSettings }: { copy: (typeof copyByLanguage)[UiLanguage]; onOpenSettings: () => void }) {
  const items = [
    { label: copy.control, icon: <SmartToyOutlined /> },
    { label: copy.ai, icon: <HexagonOutlined /> },
    { label: copy.voice, icon: <MicNone /> },
    { label: copy.settings, icon: <SettingsRounded />, onClick: onOpenSettings },
  ];

  return (
    <Paper
      sx={{
        position: 'sticky',
        bottom: 0,
        p: 0.55,
        display: 'grid',
        gridTemplateColumns: 'repeat(4, minmax(0,1fr))',
        gap: 0.5,
      }}
    >
      {items.map((item, index) => (
        <Button
          key={item.label}
          startIcon={item.icon}
          onClick={item.onClick}
          sx={{
            minHeight: 52,
            fontSize: 12,
            color: index === 0 ? '#73a5ff' : 'text.secondary',
            bgcolor: index === 0 ? '#0f1320' : 'transparent',
            boxShadow: index === 0 ? '0 10px 26px rgba(46,108,255,.18)' : 'none',
          }}
        >
          {item.label}
        </Button>
      ))}
    </Paper>
  );
}

function PagedStrip({
  children,
  columns,
  pageSize,
  itemCount,
}: {
  children: ReactNode;
  columns: string;
  pageSize: number;
  itemCount: number;
}) {
  const trackRef = useRef<HTMLDivElement | null>(null);
  const [page, setPage] = useState(0);
  const pageCount = Math.max(1, Math.ceil(itemCount / pageSize));

  return (
    <Stack spacing={0.7}>
      <Box
        ref={trackRef}
        onScroll={(event) => {
          const el = event.currentTarget;
          const nextPage =
            el.clientWidth > 0 ? Math.round(el.scrollLeft / el.clientWidth) : 0;
          if (nextPage !== page) setPage(nextPage);
        }}
        sx={{
          display: 'grid',
          gridAutoFlow: 'column',
          gridAutoColumns: columns,
          gap: 1,
          overflowX: 'auto',
          overscrollBehaviorX: 'contain',
          scrollSnapType: 'x mandatory',
          pb: 0.2,
          '& > *': { scrollSnapAlign: 'start' },
          '&::-webkit-scrollbar': { display: 'none' },
          scrollbarWidth: 'none',
        }}
      >
        {children}
      </Box>
      {pageCount > 1 ? (
        <Stack direction="row" spacing={0.6} sx={{ justifyContent: 'center' }}>
          {Array.from({ length: pageCount }).map((_, index) => (
            <Box
              key={index}
              sx={{
                width: 6,
                height: 6,
                borderRadius: '50%',
                bgcolor:
                  index === page ? 'rgba(193,207,235,.72)' : 'rgba(193,207,235,.22)',
              }}
            />
          ))}
        </Stack>
      ) : null}
    </Stack>
  );
}

function LegHeightSlider({
  label,
  value,
  onChange,
}: {
  label: string;
  value: number;
  onChange: (value: number) => void;
}) {
  return (
    <Box
      sx={{
        mt: 0.8,
        px: 0.55,
        minWidth: 0,
      }}
    >
      <Stack direction="row" sx={{ alignItems: 'center', justifyContent: 'space-between' }}>
        <Typography sx={{ color: 'text.secondary', fontSize: 10 }}>
          {label}
        </Typography>
        <Typography sx={{ color: '#80aaff', fontSize: 10, fontWeight: 700 }}>
          {Math.round(value)}%
        </Typography>
      </Stack>
      <Stack direction="row" spacing={0.6} sx={{ alignItems: 'center' }}>
        <KeyboardArrowDown sx={{ color: '#79a9ff', fontSize: 18 }} />
        <Slider
          value={value}
          min={0}
          max={100}
          step={1}
          onChange={(_, next) => onChange(Array.isArray(next) ? next[0] : next)}
          aria-label={label}
          sx={{
            flex: 1,
            '& .MuiSlider-thumb': {
              width: 18,
              height: 18,
              boxShadow: '0 0 18px rgba(72,137,255,.7)',
            },
            '& .MuiSlider-track': { height: 6 },
            '& .MuiSlider-rail': { height: 6, opacity: 0.28 },
          }}
        />
        <KeyboardArrowUp sx={{ color: '#79a9ff', fontSize: 18 }} />
      </Stack>
    </Box>
  );
}

function LeanSlider({
  label,
  value,
  onChange,
  onRelease,
}: {
  label: string;
  value: number;
  onChange: (value: number) => void;
  onRelease: () => void;
}) {
  const activeRef = useRef(false);
  const valueRef = useRef(value);
  const heartbeatRef = useRef<number | null>(null);

  useEffect(() => {
    valueRef.current = value;
  }, [value]);

  useEffect(() => () => {
    if (heartbeatRef.current !== null) {
      window.clearInterval(heartbeatRef.current);
    }
  }, []);

  const startHeartbeat = () => {
    activeRef.current = true;
    if (heartbeatRef.current !== null) return;
    heartbeatRef.current = window.setInterval(() => {
      if (activeRef.current) {
        onChange(valueRef.current);
      }
    }, 120);
  };

  const release = () => {
    if (!activeRef.current) return;
    activeRef.current = false;
    if (heartbeatRef.current !== null) {
      window.clearInterval(heartbeatRef.current);
      heartbeatRef.current = null;
    }
    onRelease();
  };

  return (
    <Box
      sx={{ mt: 0.8, px: 0.55 }}
      onPointerDown={startHeartbeat}
      onPointerUp={release}
      onPointerCancel={release}
      onLostPointerCapture={release}
      onTouchEnd={release}
      onMouseUp={release}
    >
      <Stack direction="row" sx={{ alignItems: 'center', justifyContent: 'space-between' }}>
        <Typography sx={{ color: 'text.secondary', fontSize: 10 }}>
          {label}
        </Typography>
        <Typography sx={{ color: '#80aaff', fontSize: 10, fontWeight: 700 }}>
          {value > 0 ? `R ${value}%` : value < 0 ? `L ${Math.abs(value)}%` : '0%'}
        </Typography>
      </Stack>
      <Stack direction="row" spacing={0.6} sx={{ alignItems: 'center' }}>
        <KeyboardArrowLeft sx={{ color: '#79a9ff', fontSize: 18 }} />
        <Slider
          value={value}
          min={-100}
          max={100}
          step={1}
          marks={[{ value: 0 }]}
          onChange={(_, next) => {
            const nextValue = Array.isArray(next) ? next[0] : next;
            valueRef.current = nextValue;
            onChange(nextValue);
          }}
          aria-label={label}
          sx={{
            flex: 1,
            '& .MuiSlider-thumb': {
              width: 18,
              height: 18,
              boxShadow: '0 0 18px rgba(72,137,255,.7)',
            },
            '& .MuiSlider-track': { height: 6 },
            '& .MuiSlider-rail': { height: 6, opacity: 0.28 },
            '& .MuiSlider-mark': {
              width: 2,
              height: 12,
              bgcolor: 'rgba(255,255,255,.55)',
            },
          }}
        />
        <KeyboardArrowRight sx={{ color: '#79a9ff', fontSize: 18 }} />
      </Stack>
    </Box>
  );
}

function SmallActionButton({
  children,
  onClick,
  disabled = false,
}: {
  children: ReactNode;
  onClick: () => void | Promise<void>;
  disabled?: boolean;
}) {
  return (
    <Button
      fullWidth
      variant="outlined"
      onClick={onClick}
      disabled={disabled}
      sx={{ minHeight: 42, borderRadius: 2.5 }}
    >
      {children}
    </Button>
  );
}

const miniRoundButtonSx = {
  width: 36,
  height: 36,
  borderRadius: 999,
  bgcolor: '#0f1320',
  border: '1px solid #263041',
};





