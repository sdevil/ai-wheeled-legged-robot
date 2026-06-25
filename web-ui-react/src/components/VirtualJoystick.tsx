import { Box, Slider, Stack, Typography } from '@mui/material';
import { type ReactNode, useEffect, useMemo, useRef, useState } from 'react';

interface VirtualJoystickProps {
  title: string;
  footerLabel: string;
  speedValue: number;
  onSpeedChange: (value: number) => void;
  onMove: (x: number, y: number) => void;
  onEnd: () => void;
  speedLabels: { low: string; medium: string; high: string; extreme?: string };
  beforeFooter?: ReactNode;
}

export function VirtualJoystick({
  title,
  footerLabel,
  speedValue,
  onSpeedChange,
  onMove,
  onEnd,
  speedLabels,
  beforeFooter,
}: VirtualJoystickProps) {
  const surfaceRef = useRef<HTMLDivElement | null>(null);
  const activePointerIdRef = useRef<number | null>(null);
  const latestVectorRef = useRef({ x: 0, y: 0 });
  const heartbeatRef = useRef<number | null>(null);
  const lastMoveSentAtRef = useRef(0);
  const onEndRef = useRef(onEnd);
  const [knob, setKnob] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);

  useEffect(() => {
    onEndRef.current = onEnd;
  }, [onEnd]);

  useEffect(() => () => {
    if (heartbeatRef.current !== null) window.clearInterval(heartbeatRef.current);
    if (activePointerIdRef.current !== null) onEndRef.current();
  }, []);

  const speedLabel = useMemo(() => {
    if (speedValue <= 35) return speedLabels.low;
    if (speedValue <= 70) return speedLabels.medium;
    if (speedValue <= 100 || !speedLabels.extreme) return speedLabels.high;
    return speedLabels.extreme;
  }, [speedLabels.extreme, speedLabels.high, speedLabels.low, speedLabels.medium, speedValue]);

  const handlePointer = (clientX: number, clientY: number) => {
    const surface = surfaceRef.current;
    if (!surface) return;
    const rect = surface.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const dx = clientX - cx;
    const dy = clientY - cy;
    const radius = rect.width * 0.38;
    const distance = Math.hypot(dx, dy);
    const factor = distance > radius ? radius / distance : 1;
    const x = dx * factor;
    const y = dy * factor;
    setKnob({ x, y });
    latestVectorRef.current = { x: x / radius, y: -(y / radius) };
  };

  const startHeartbeat = () => {
    if (heartbeatRef.current !== null) window.clearInterval(heartbeatRef.current);
    heartbeatRef.current = window.setInterval(() => {
      const vector = latestVectorRef.current;
      onMove(vector.x, vector.y);
    }, 120);
  };

  const sendLatestMove = () => {
    const now = performance.now();
    if (now - lastMoveSentAtRef.current < 48) return;
    lastMoveSentAtRef.current = now;
    const vector = latestVectorRef.current;
    onMove(vector.x, vector.y);
  };

  const reset = () => {
    if (activePointerIdRef.current === null) return;
    activePointerIdRef.current = null;
    if (heartbeatRef.current !== null) {
      window.clearInterval(heartbeatRef.current);
      heartbeatRef.current = null;
    }
    latestVectorRef.current = { x: 0, y: 0 };
    setDragging(false);
    setKnob({ x: 0, y: 0 });
    onEnd();
  };

  return (
    <Stack spacing={1.05} sx={{ height: '100%' }}>
      <Typography sx={{ fontSize: 15, fontWeight: 700 }}>{title}</Typography>
      <Box
        ref={surfaceRef}
        onPointerDown={(event) => {
          event.preventDefault();
          if (activePointerIdRef.current !== null) return;
          activePointerIdRef.current = event.pointerId;
          setDragging(true);
          event.currentTarget.setPointerCapture(event.pointerId);
          handlePointer(event.clientX, event.clientY);
          onMove(latestVectorRef.current.x, latestVectorRef.current.y);
          startHeartbeat();
        }}
        onPointerMove={(event) => {
          event.preventDefault();
          if (activePointerIdRef.current !== event.pointerId) return;
          handlePointer(event.clientX, event.clientY);
          sendLatestMove();
        }}
        onPointerUp={(event) => {
          if (activePointerIdRef.current === event.pointerId) {
            reset();
          }
        }}
        onPointerCancel={reset}
        onPointerLeave={(event) => {
          if (activePointerIdRef.current === event.pointerId && event.buttons === 0) {
            reset();
          }
        }}
        onLostPointerCapture={reset}
        onContextMenu={(event) => event.preventDefault()}
        sx={{
          position: 'relative',
          width: '100%',
          minHeight: 'clamp(128px, calc((min(100vw, 720px) - 54px) / 2), 310px)',
          flexShrink: 0,
          aspectRatio: '1 / 1',
          borderRadius: '50%',
          overflow: 'hidden',
          touchAction: 'none',
          overscrollBehavior: 'contain',
          userSelect: 'none',
          WebkitUserSelect: 'none',
          background:
            'radial-gradient(circle at center, rgba(19,26,40,1) 0%, rgba(11,16,26,1) 62%, rgba(9,13,20,1) 100%)',
          border: '1px solid rgba(82,98,134,.18)',
        }}
      >
        <Box sx={ringSx(0.92)} />
        <Box sx={ringSx(0.38, 'rgba(77,141,255,.10)')} />
        <ArrowMarker direction="up" />
        <ArrowMarker direction="right" />
        <ArrowMarker direction="down" />
        <ArrowMarker direction="left" />
        <Box
          sx={{
            position: 'absolute',
            inset: 0,
            display: 'grid',
            placeItems: 'center',
          }}
        >
          <Box
            sx={{
              width: '24%',
              height: '24%',
              borderRadius: '50%',
              background:
                'radial-gradient(circle at 35% 35%, #6ba6ff 0%, #2e6bff 72%)',
              boxShadow: '0 0 32px rgba(77,141,255,.52)',
              transform: `translate(${knob.x}px, ${knob.y}px)`,
              transition: dragging ? 'none' : 'transform 45ms linear',
            }}
          />
        </Box>
      </Box>
      {beforeFooter}
      <Box
        sx={{
          borderRadius: 1.4,
          bgcolor: '#0d1220',
          px: 0.85,
          py: 0.72,
        }}
      >
        <Stack direction="row" spacing={1.1} sx={{ alignItems: 'center' }}>
          <Typography sx={{ color: '#79a9ff', fontWeight: 700, fontSize: 13 }}>
            {footerLabel}
          </Typography>
          <Slider
            value={speedValue}
            min={15}
            max={120}
            onChange={(_, value) => onSpeedChange(value as number)}
            sx={{
              color: '#4d8dff',
              flex: 1,
            }}
          />
          <Typography sx={{ fontWeight: 700, fontSize: 13 }}>{speedLabel}</Typography>
        </Stack>
      </Box>
    </Stack>
  );
}

function ringSx(scale: number, borderColor = 'rgba(101,126,170,.18)') {
  return {
    position: 'absolute',
    width: `${scale * 100}%`,
    height: `${scale * 100}%`,
    top: `${(1 - scale) * 50}%`,
    left: `${(1 - scale) * 50}%`,
    borderRadius: '50%',
    border: `1px solid ${borderColor}`,
  };
}

function ArrowMarker({
  direction,
}: {
  direction: 'up' | 'right' | 'down' | 'left';
}) {
  const positionSx =
    direction === 'up'
      ? { top: '11%', left: '50%', transform: 'translateX(-50%) rotate(0deg)' }
      : direction === 'right'
        ? { top: '50%', right: '11%', transform: 'translateY(-50%) rotate(90deg)' }
        : direction === 'down'
          ? { bottom: '11%', left: '50%', transform: 'translateX(-50%) rotate(180deg)' }
          : { top: '50%', left: '11%', transform: 'translateY(-50%) rotate(-90deg)' };

  return (
    <Box
      sx={{
        position: 'absolute',
        width: 18,
        height: 18,
        display: 'grid',
        placeItems: 'center',
        pointerEvents: 'none',
        ...positionSx,
      }}
    >
      <Box
        sx={{
          width: 12,
          height: 12,
          borderTop: '2px solid #79a9ff',
          borderLeft: '2px solid #79a9ff',
          transform: 'rotate(45deg)',
          boxShadow: '0 0 10px rgba(77,141,255,.22)',
        }}
      />
    </Box>
  );
}

