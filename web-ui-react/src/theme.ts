import { alpha, createTheme } from '@mui/material/styles';

export const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: '#4d8dff' },
    secondary: { main: '#7f69ff' },
    success: { main: '#39df7c' },
    error: { main: '#ff7b7b' },
    background: {
      default: '#07090d',
      paper: '#10141c',
    },
    text: {
      primary: '#eef3ff',
      secondary: '#97a3b7',
    },
  },
  shape: {
    borderRadius: 20,
  },
  typography: {
    fontFamily: `Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif`,
    h4: { fontWeight: 800, letterSpacing: 0 },
    h6: { fontWeight: 700, letterSpacing: 0 },
    button: { fontWeight: 700, letterSpacing: 0 },
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        html: { height: '100%' },
        body: {
          minHeight: '100%',
          background:
            'radial-gradient(circle at top, #111827 0%, #07090d 42%)',
        },
        '#root': { minHeight: '100%' },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundImage:
            'linear-gradient(180deg, rgba(19,24,34,.97), rgba(10,13,19,.98))',
          border: '1px solid #263041',
          boxShadow: '0 22px 50px rgba(0,0,0,.34)',
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: { borderRadius: 16, textTransform: 'none' },
      },
    },
    MuiChip: {
      styleOverrides: {
        root: {
          borderRadius: 14,
          backgroundColor: alpha('#10141c', 0.88),
          border: '1px solid rgba(255,255,255,.06)',
        },
      },
    },
    MuiCard: {
      styleOverrides: {
        root: {
          backgroundImage:
            'linear-gradient(180deg, rgba(19,24,34,.97), rgba(10,13,19,.98))',
          border: '1px solid #263041',
          boxShadow: '0 22px 50px rgba(0,0,0,.34)',
        },
      },
    },
  },
});
