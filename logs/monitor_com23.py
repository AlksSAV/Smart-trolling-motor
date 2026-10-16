#!/usr/bin/env python3
"""Монитор COM23 с отправкой команд (однопоточный, Windows-friendly).

Выводит данные с платы и периодически проверяет ввод с клавиатуры.

Использование:
  python logs/monitor_com23.py [COM-порт]
  
Команды: M / H / A / S / L  (Q — выход)
"""

import sys, time, select
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM23"
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(2)

    print("=" * 60)
    print(f"  Монитор {PORT} (115200)")
    print("  Вводи: M / H / A / S / L  (Q — выход)")
    print("  Вывод с платы появляется автоматически")
    print("=" * 60)

    buf = b""
    while True:
        # --- Чтение порта ---
        try:
            chunk = ser.read(ser.in_waiting or 256)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
        except Exception:
            pass

        # --- Чтение клавиатуры (non-blocking) ---
        # Windows: select на sys.stdin не работает, используем msvcrt
        try:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch().decode("utf-8", errors="replace").strip().upper()
                if ch == "Q":
                    break
                elif ch in ("M", "H", "A", "S", "L"):
                    ser.write((ch + "\n").encode())
                else:
                    sys.stdout.write(f"\n  -> Команда: {ch}. Доступно: M/H/A/S/L (Q — выход)\n")
                    sys.stdout.flush()
        except ImportError:
            # Unix fallback: short timeout input
            pass

        time.sleep(0.01)

    ser.close()
    print("\n  Завершено.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Прервано.")
