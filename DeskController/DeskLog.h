// =====================================================================
//  DeskLog.h  —  serial output that also remembers the last line
//
//  Everything the controller prints goes through `Out`. It is written to
//  the serial port as before, and the last complete line is kept so the
//  Home Assistant "Last message" sensor can show replies, prompts
//  (e.g. during calibration) and errors on the web UI.
// =====================================================================
#pragma once
#include <Arduino.h>
#include <stdarg.h>

class DeskLog {
public:
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) write(buf);
  }
  void print(const char* s)          { write(s); }
  void println(const char* s = "")   { write(s); write("\n"); }
  void println(const String& s)      { println(s.c_str()); }

  // Copies the last complete line into 'out'. Returns true if it's new
  // since the previous call.
  bool takeLastLine(char* out, size_t n) {
    portENTER_CRITICAL(&_mux);
    bool fresh = _new; _new = false;
    strncpy(out, _last, n - 1); out[n - 1] = 0;
    portEXIT_CRITICAL(&_mux);
    return fresh;
  }

private:
  void write(const char* s) {
    Serial.print(s);
    portENTER_CRITICAL(&_mux);
    for (; *s; s++) {
      if (*s == '\n') {
        if (_len) { memcpy(_last, _cur, _len); _last[_len] = 0; _new = true; }
        _len = 0;
      } else if (*s != '\r' && _len < sizeof(_cur) - 1) {
        if (_len == 0 && (*s == ' ' || *s == '=')) continue;   // trim leading padding
        _cur[_len++] = *s;
      }
    }
    portEXIT_CRITICAL(&_mux);
  }
  char   _cur[200], _last[200] = "";
  size_t _len = 0;
  bool   _new = false;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};
