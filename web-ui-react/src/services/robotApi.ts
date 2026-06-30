import type {
  DashboardModel,
  DiagnosticsSnapshot,
  RobotSettings,
  RobotStatus,
  TransportMode,
} from '../types';

const mockModel = (host: string): DashboardModel => ({
  bootId: 0,
  firmwareVersion: '',
  firmwareBuild: '',
  cameraFirmwareVersion: '',
  cameraFirmwareBuild: '',
  cameraStatusAgeMs: 0,
  cameraVersionStale: true,
  online: false,
  robotHost: host,
  robotIp: host,
  robotState: 'Offline',
  robotNetState: 'OFFLINE',
  batteryPercent: 0,
  batteryVoltage: 0,
  legHeightPercent: 50,
  legLeanPercent: 0,
  guardServoAngle: 0,
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
  cameraState: 'OFFLINE',
  cameraMessage: '',
  cameraResolution: '',
  cameraIp: '',
  cameraUrl: '',
  clients: 0,
  controlOwnerPresent: false,
  controlOwner: true,
  otaRunning: false,
  otaProgress: 0,
  otaMessage: '',
  oledFace: '(-_-)',
  activeMode: '',
  maintenanceMode: false,
  controlTransport: 'Offline',
});

export type SaveResult = { ok: boolean; message: string; connectionLost?: boolean };
type AckWaiter = { resolve: (ok: boolean) => void; timer: number };

function getOrCreateClientId() {
  const key = 'wrobot.clientId';
  const existing = localStorage.getItem(key);
  if (existing) return existing;
  const created = `client-${Math.random().toString(36).slice(2, 10)}-${Date.now().toString(36)}`;
  localStorage.setItem(key, created);
  return created;
}

export class RobotApi {
  private ws?: WebSocket;
  private wsReady = false;
  private wsConnectTimer: number | null = null;
  private wsReconnectTimer: number | null = null;
  private suppressCloseNotice = false;
  private websocketClosedHandler?: () => void;
  private socketReadyPromise?: Promise<boolean>;
  private resolveSocketReady?: (ready: boolean) => void;
  private host: string;
  private transportMode: TransportMode;
  private requestId = 1;
  private ackWaiters = new Map<number, AckWaiter>();
  private readonly clientId = getOrCreateClientId();

  constructor(host: string, transportMode: TransportMode) {
    this.host = host;
    this.transportMode = transportMode;
  }

  setHost(host: string) {
    if (host === this.host) return;
    this.host = host;
    this.disposeSocket();
  }

  setTransport(mode: TransportMode) {
    if (mode === this.transportMode) return;
    this.transportMode = mode;
    this.disposeSocket();
  }

  onWebsocketClosed(handler: (() => void) | undefined) {
    this.websocketClosedHandler = handler;
  }

  openControlChannel() {
    void this.ensureSocket();
  }

  private getBaseHttp() {
    return `http://${this.host}`;
  }

  private withClientId(path: string) {
    const url = new URL(`${this.getBaseHttp()}${path}`);
    url.searchParams.set('client_id', this.clientId);
    return url.toString();
  }

  private getWsUrl() {
    const url = new URL(`ws://${this.host}:81/ws`);
    url.searchParams.set('client_id', this.clientId);
    return url.toString();
  }

  private transportLabel() {
    if (this.transportMode === 'http') return 'HTTP';
    return 'WebSocket';
  }

  private cameraStatusUrl(cameraUrl: string) {
    const trimmed = cameraUrl.trim();
    if (!trimmed) return '';
    try {
      const url = new URL(trimmed);
      url.pathname = '/api/status';
      url.search = '';
      url.hash = '';
      return url.toString();
    } catch {
      const base = trimmed.replace(/\/(?:stream\.mjpg|snapshot\.jpg)?(?:\?.*)?$/, '');
      return `${base}/api/status`;
    }
  }

  private async fetchCameraRuntimeStatus(cameraUrl: string, signal?: AbortSignal) {
    const statusUrl = this.cameraStatusUrl(cameraUrl);
    if (!statusUrl) return null;
    try {
      const response = await this.fetchWithTimeout(statusUrl, { cache: 'no-store' }, 450, signal);
      if (!response.ok) return null;
      const data = await response.json() as {
        label?: string;
        count?: number;
        version?: string;
        resolution?: string;
      };
      return data;
    } catch {
      return null;
    }
  }

  private async fetchWithTimeout(
    input: RequestInfo | URL,
    init: RequestInit,
    timeoutMs: number,
    parentSignal?: AbortSignal,
  ) {
    const controller = new AbortController();
    const abortFromParent = () => controller.abort();
    if (parentSignal?.aborted) controller.abort();
    else parentSignal?.addEventListener('abort', abortFromParent, { once: true });
    const timer = window.setTimeout(() => controller.abort(), timeoutMs);
    try {
      return await fetch(input, { ...init, signal: controller.signal });
    } finally {
      window.clearTimeout(timer);
      parentSignal?.removeEventListener('abort', abortFromParent);
    }
  }

  async fetchStatus(cameraUrlOverride = '', signal?: AbortSignal): Promise<DashboardModel> {
    try {
      const response = await this.fetchWithTimeout(this.withClientId('/api/status'), {
        cache: 'no-store',
      }, 1800, signal);
      if (!response.ok) throw new Error(String(response.status));
      const data = (await response.json()) as RobotStatus;
      const controlOwnerPresent = data.control_owner_present === true;
      const controlOwner = data.control_owner !== false;
      if (!controlOwnerPresent || controlOwner) void this.ensureSocket();
      else this.disposeSocket();
      const cameraUrl = data.camera_net_ip
        ? `http://${data.camera_net_ip}:8080/stream.mjpg`
        : data.camera_url || cameraUrlOverride;
      const cameraRuntime = await this.fetchCameraRuntimeStatus(cameraUrl, signal);
      const targetLabelRaw = cameraRuntime?.label || data.camera_detect_label || 'NO_TARGET';
      const targetCount = cameraRuntime?.count ?? data.camera_detect_count ?? 0;
      const cameraResolution = cameraRuntime?.resolution || data.camera_resolution || '';
      const activeMode = data.active_mode || '';
      const robotState = data.sitting
        ? 'Sitting'
        : activeMode === 'track_mode'
          ? 'Track'
          : activeMode === 'waltz_show'
            ? 'Waltz'
            : data.enabled
              ? 'Active'
              : 'Standby';
      const aiMode = activeMode === 'track_mode'
        ? 'Track'
        : activeMode === 'waltz_show'
          ? 'Waltz'
          : data.enabled
            ? 'Active'
            : 'Standby';
      return {
        bootId: data.boot_id || 0,
        firmwareVersion: data.firmware_version || '',
        firmwareBuild: data.firmware_build || '',
        cameraFirmwareVersion: data.camera_firmware_version || '',
        cameraFirmwareBuild: data.camera_firmware_build || '',
        cameraStatusAgeMs: data.camera_status_age_ms || 0,
        cameraVersionStale: data.camera_version_stale !== false,
        online: true,
        robotHost: this.host,
        robotIp: data.robot_sta_ip || this.host,
        robotState,
        robotNetState: data.robot_net_state,
        batteryPercent: data.battery_percent,
        batteryVoltage: data.battery_voltage,
        legHeightPercent: data.leg_height_percent ?? 50,
        legLeanPercent: data.leg_lean_percent ?? 0,
        guardServoAngle: data.guard_angle ?? 0,
        wifiDbm: 0,
        fps: 0,
        latencyMs: 0,
        aiMode,
        targetLabelRaw,
        targetLabel: targetLabelRaw === 'NO_TARGET'
          ? 'No Target'
          : `${formatDetectionLabel(targetLabelRaw)}${targetCount > 1 ? ` x${targetCount}` : ''}`,
        speedMps: 0,
        pitch: data.angle,
        roll: 0,
        temperatureC: 0,
        cameraState: data.camera_net_state,
        cameraMessage: data.camera_net_message,
        cameraResolution,
        cameraIp: data.camera_net_ip,
        cameraUrl,
        clients: data.clients,
        controlOwnerPresent,
        controlOwner,
        activeMode,
        maintenanceMode: data.maintenance,
        otaRunning: data.ota_running,
        otaProgress: data.ota_progress,
        otaMessage: data.ota_message || '',
        oledFace: data.enabled ? '(^-^)' : '(-_-)',
        controlTransport: this.transportLabel(),
      };
    } catch (error) {
      if (error instanceof DOMException && error.name === 'AbortError' && signal?.aborted) {
        throw error;
      }
      return { ...mockModel(this.host), cameraUrl: cameraUrlOverride };
    }
  }

  async fetchSettings(signal?: AbortSignal): Promise<RobotSettings | null> {
    try {
      const response = await this.fetchWithTimeout(this.withClientId('/api/settings'), {
        cache: 'no-store',
      }, 2500, signal);
      if (!response.ok) throw new Error(String(response.status));
      return (await response.json()) as RobotSettings;
    } catch {
      return null;
    }
  }

  async fetchDiagnostics(signal?: AbortSignal): Promise<DiagnosticsSnapshot | null> {
    try {
      const response = await this.fetchWithTimeout(this.withClientId('/api/diagnostics'), {
        cache: 'no-store',
      }, 1800, signal);
      if (!response.ok) throw new Error(String(response.status));
      return (await response.json()) as DiagnosticsSnapshot;
    } catch {
      return null;
    }
  }

  private resolveAllAcks(ok: boolean) {
    this.ackWaiters.forEach(({ resolve, timer }) => {
      window.clearTimeout(timer);
      resolve(ok);
    });
    this.ackWaiters.clear();
  }

  private async ensureSocket() {
    if (this.transportMode === 'http') return false;
    if (this.wsReady && this.ws?.readyState === WebSocket.OPEN) return true;
    if (this.ws && this.ws.readyState === WebSocket.CONNECTING && this.socketReadyPromise) {
      return this.socketReadyPromise;
    }
    if (this.wsReconnectTimer !== null) {
      window.clearTimeout(this.wsReconnectTimer);
      this.wsReconnectTimer = null;
    }
    try {
      const socket = new WebSocket(this.getWsUrl());
      this.ws = socket;
      this.suppressCloseNotice = false;
      this.socketReadyPromise = new Promise<boolean>((resolve) => {
        this.resolveSocketReady = resolve;
      });
      this.wsConnectTimer = window.setTimeout(() => {
        if (this.ws === socket && socket.readyState === WebSocket.CONNECTING) socket.close();
      }, 5000);
      socket.onopen = () => {
        if (this.ws !== socket) return;
        if (this.wsConnectTimer !== null) window.clearTimeout(this.wsConnectTimer);
        this.wsConnectTimer = null;
        this.wsReady = true;
        this.resolveSocketReady?.(true);
        this.resolveSocketReady = undefined;
      };
      socket.onmessage = (event) => {
        if (this.ws !== socket) return;
        try {
          const message = JSON.parse(String(event.data)) as { id?: number; ok?: boolean };
          if (typeof message.id !== 'number') return;
          const waiter = this.ackWaiters.get(message.id);
          if (!waiter) return;
          window.clearTimeout(waiter.timer);
          this.ackWaiters.delete(message.id);
          waiter.resolve(message.ok === true);
        } catch {
          // Ignore non-JSON diagnostic frames.
        }
      };
      socket.onclose = () => {
        if (this.ws !== socket) return;
        const wasReady = this.wsReady;
        const shouldNotify =
          wasReady &&
          !this.suppressCloseNotice &&
          this.transportMode !== 'http' &&
          document.visibilityState === 'visible';
        if (this.wsConnectTimer !== null) window.clearTimeout(this.wsConnectTimer);
        this.wsConnectTimer = null;
        this.wsReady = false;
        this.ws = undefined;
        this.resolveSocketReady?.(false);
        this.resolveSocketReady = undefined;
        this.socketReadyPromise = undefined;
        this.resolveAllAcks(false);
        if (shouldNotify) this.websocketClosedHandler?.();
        if (this.transportMode !== 'http' && document.visibilityState === 'visible') {
          this.wsReconnectTimer = window.setTimeout(() => {
            this.wsReconnectTimer = null;
            void this.ensureSocket();
          }, 600);
        }
      };
      socket.onerror = () => {
        if (this.ws !== socket) return;
        this.wsReady = false;
        socket.close();
      };
      return this.socketReadyPromise;
    } catch {
      this.wsReady = false;
      this.resolveSocketReady?.(false);
      this.resolveSocketReady = undefined;
      this.socketReadyPromise = undefined;
      return false;
    }
  }

  private async post(path: string, params: Record<string, string> = {}, timeoutMs = 800) {
    const url = new URL(this.withClientId(path));
    Object.entries(params).forEach(([key, value]) => url.searchParams.set(key, value));
    const controller = new AbortController();
    const timer = window.setTimeout(() => controller.abort(), timeoutMs);
    try {
      return await fetch(url, { method: 'POST', cache: 'no-store', signal: controller.signal });
    } finally {
      window.clearTimeout(timer);
    }
  }

  private async postForm(path: string, params: Record<string, string> = {}) {
    return this.fetchWithTimeout(this.withClientId(path), {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams(params),
    }, 8000);
  }

  private sendSocketNow(payload: unknown) {
    if (this.transportMode === 'http') return false;
    if (this.wsReady && this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(payload));
      return true;
    }
    void this.ensureSocket();
    return false;
  }

  private async sendSocketRequest(payload: Record<string, unknown>) {
    const ready = await this.ensureSocket();
    if (!ready || !this.wsReady || this.ws?.readyState !== WebSocket.OPEN) return null;
    const id = this.requestId++;
    const promise = new Promise<boolean>((resolve) => {
      const timer = window.setTimeout(() => {
        this.ackWaiters.delete(id);
        resolve(false);
      }, 800);
      this.ackWaiters.set(id, { resolve, timer });
    });
    this.ws.send(JSON.stringify({ ...payload, id }));
    return promise;
  }


  async saveAllSettings(params: {
    robotName: string;
    uiLanguage: string;
    controlMode: string;
    gamepadMac: string;
    cameraUrl: string;
    cameraResolution: string;
    robotWifiSsid: string;
    robotWifiPassword: string;
    applyWifi: boolean;
  }): Promise<SaveResult> {
    try {
      const form: Record<string, string> = {
        robot_name: params.robotName,
        ui_language: params.uiLanguage,
        control_mode: params.controlMode,
        gamepad_mac: params.gamepadMac,
        camera_url: params.cameraUrl,
        camera_resolution: params.cameraResolution,
        robot_wifi_ssid: params.robotWifiSsid,
      };
      if (params.robotWifiPassword) form.robot_wifi_password = params.robotWifiPassword;
      const response = await this.postForm('/api/settings', form);
      if (!response.ok) return { ok: false, message: 'Save failed' };
      if (params.controlMode === 'gamepad') {
        return { ok: true, message: 'Saved, rebooting to Gamepad mode' };
      }
      if (params.applyWifi && params.robotWifiSsid.trim()) {
        const wifiForm: Record<string, string> = { robot_wifi_ssid: params.robotWifiSsid };
        if (params.robotWifiPassword) wifiForm.robot_wifi_password = params.robotWifiPassword;
        try {
          const wifiResponse = await this.postForm('/api/robot/connect_wifi', wifiForm);
          if (!wifiResponse.ok) return { ok: true, message: 'Saved; robot WiFi apply failed' };
        } catch {
          return { ok: true, message: 'Saved; robot WiFi apply timed out' };
        }
        try {
          const cameraResponse = await this.postForm('/api/camera/apply_wifi');
          if (!cameraResponse.ok) return { ok: true, message: 'Saved; camera sync failed' };
        } catch {
          return { ok: true, message: 'Saved; camera sync timed out' };
        }
        return { ok: true, message: 'Saved and applying' };
      }
      return { ok: true, message: 'Saved' };
    } catch {
      return { ok: false, message: 'Save failed' };
    }
  }

  sendDrive(x: number, y: number, speed: number) {
    const curveAxis = (value: number) => {
      const magnitude = Math.abs(value);
      if (magnitude < 0.035) return 0;
      return Math.sign(value) * Math.pow(magnitude, 1.45);
    };
    const speedScale = Math.max(15, Math.min(120, speed)) / 100;
    const payload = {
      x: Math.round(curveAxis(x) * 72 * speedScale),
      y: Math.round(curveAxis(y) * 100 * speedScale),
    };
    this.sendSocketNow({ type: 'drive', ...payload });
  }

  sendTrackDistanceAdjust(value: number) {
    const adjusted = Math.max(-100, Math.min(100, Math.round(value)));
    this.sendSocketNow({ type: 'track_distance', value: adjusted });
  }

  sendGimbal(x: number, y: number, speed: number) {
    const curveAxis = (value: number) => {
      const magnitude = Math.abs(value);
      if (magnitude < 0.035) return 0;
      return Math.sign(value) * Math.pow(magnitude, 1.35);
    };
    const speedScale = Math.max(15, Math.min(120, speed)) / 100;
    const yaw = Math.round(curveAxis(x) * 72 * speedScale);
    const pitchDelta = Math.round(curveAxis(y) * 4 * speedScale);
    this.sendSocketNow({ type: 'gimbal', yaw, pitchDelta });
  }

  sendGuardServo(angle: number) {
    const value = Math.max(0, Math.min(180, Math.round(angle)));
    this.sendSocketNow({ type: 'guard_servo', angle: value });
  }

  sendLegHeight(direction: number) {
    const value = Math.max(-1, Math.min(1, Math.round(direction)));
    this.sendSocketNow({ type: 'leg_height', direction: value });
  }

  sendLegHeightValue(percent: number) {
    const value = Math.max(0, Math.min(100, Math.round(percent)));
    this.sendSocketNow({ type: 'leg_height_value', percent: value });
  }

  sendLegLean(percent: number, source = 'ui') {
    const value = Math.max(-100, Math.min(100, Math.round(percent)));
    this.sendSocketNow({ type: 'leg_lean', percent: value, source });
  }

  emergencyStop() {
    this.sendSocketNow({ type: 'drive', x: 0, y: 0 });
    this.sendSocketNow({ type: 'leg_height', direction: 0 });
    this.sendSocketNow({ type: 'leg_lean', percent: 0, source: 'emergency_stop' });
    const url = `${this.withClientId('/api/drive')}&x=0&y=0`;
    const legHeightUrl = `${this.withClientId('/api/legs/height')}&direction=0`;
    const legLeanUrl = `${this.withClientId('/api/legs/lean')}&percent=0`;
    if (navigator.sendBeacon) navigator.sendBeacon(url);
    else void fetch(url, { method: 'POST', keepalive: true });
    if (navigator.sendBeacon) {
      navigator.sendBeacon(legHeightUrl);
      navigator.sendBeacon(legLeanUrl);
    } else {
      void fetch(legHeightUrl, { method: 'POST', keepalive: true });
      void fetch(legLeanUrl, { method: 'POST', keepalive: true });
    }
  }

  async sendTrackSelection(
    selection: { x: number; y: number; w: number; h: number },
    profile = 0,
  ) {
    const ack = await this.sendSocketRequest({ type: 'track_roi', ...selection, profile });
    return ack ? ack : false;
  }

  async sendTrackUnlock() {
    return this.sendTrackScan();
  }

  async sendTrackScan() {
    const ack = await this.sendSocketRequest({ type: 'track_unlock' });
    return ack ? ack : false;
  }

  async sendAction(name: string) {
    const ack = await this.sendSocketRequest({ type: 'action', name });
    if (ack) return ack;
    try {
      const response = await this.post('/api/action', { name }, 1800);
      return response.ok;
    } catch {
      return false;
    }
  }

  async takeOverControl() {
    try {
      const response = await this.post('/api/control/takeover', {}, 1800);
      if (!response.ok) return false;
      this.disposeSocket();
      return await this.ensureSocket();
    } catch {
      return false;
    }
  }

  uploadFirmware(file: File, onProgress: (percent: number) => void, usbPowered = false) {
    return new Promise<SaveResult>((resolve) => {
      const request = new XMLHttpRequest();
      const query = new URLSearchParams({
        usb_powered: usbPowered ? '1' : '0',
        size: String(file.size),
      });
      const uploadUrl = new URL(this.withClientId('/api/ota/upload'));
      query.forEach((value, key) => uploadUrl.searchParams.set(key, value));
      request.open('POST', uploadUrl.toString());
      request.upload.onprogress = (event) => {
        if (event.lengthComputable) onProgress(Math.round((event.loaded * 100) / event.total));
      };
      request.onload = () => {
        const ok = request.status >= 200 && request.status < 300;
        if (ok) onProgress(100);
        resolve({
          ok,
          message: ok ? 'OTA uploaded, rebooting' : request.responseText || 'OTA failed',
        });
      };
      request.onerror = () => resolve({
        ok: false,
        connectionLost: true,
        message: 'OTA response interrupted; verifying robot restart',
      });
      const form = new FormData();
      form.append('firmware', file, file.name);
      request.send(form);
    });
  }

  async waitForRestart(
    previousBootId: number,
    previousFirmwareVersion = '',
    timeoutMs = 75000,
  ) {
    const startedAt = Date.now();
    let observedOffline = false;
    while (Date.now() - startedAt < timeoutMs) {
      await new Promise((resolve) => window.setTimeout(resolve, 800));
      const status = await this.fetchStatus();
      if (!status.online) {
        observedOffline = true;
        continue;
      }
      const bootIdChanged = previousBootId > 0 &&
        status.bootId > 0 && status.bootId !== previousBootId;
      const legacyRestartDetected = previousBootId <= 0 && observedOffline;
      if (bootIdChanged || legacyRestartDetected) {
        if (
          previousFirmwareVersion &&
          status.firmwareVersion &&
          status.firmwareVersion === previousFirmwareVersion
        ) {
          return { ...status, otaMessage: 'Robot restarted, but firmware version did not change' };
        }
        return status;
      }
    }
    return null;
  }

  async rollbackFirmware(usbPowered = false): Promise<SaveResult> {
    try {
      const response = await this.post('/api/ota/rollback', {
        usb_powered: usbPowered ? '1' : '0',
      });
      return { ok: response.ok, message: response.ok ? 'Rollback scheduled' : await response.text() };
    } catch {
      return { ok: false, message: 'Rollback failed' };
    }
  }

  disposeSocket() {
    if (this.wsConnectTimer !== null) window.clearTimeout(this.wsConnectTimer);
    this.wsConnectTimer = null;
    if (this.wsReconnectTimer !== null) window.clearTimeout(this.wsReconnectTimer);
    this.wsReconnectTimer = null;
    this.resolveSocketReady?.(false);
    this.resolveSocketReady = undefined;
    this.socketReadyPromise = undefined;
    this.resolveAllAcks(false);
    if (this.ws) {
      this.suppressCloseNotice = true;
      this.ws.close();
      this.ws = undefined;
    }
    this.wsReady = false;
  }
}

function formatDetectionLabel(label: string) {
  return label.toLowerCase().split('_')
    .map((part) => part ? part[0].toUpperCase() + part.slice(1) : '')
    .join(' ');
}
