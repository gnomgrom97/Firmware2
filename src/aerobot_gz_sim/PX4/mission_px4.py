#!/usr/bin/env python3
import asyncio
from mavsdk_grpc import System

# ===== НАСТРОЙКИ =====
ALT_REL    = 1.5      # высота взлёта над землёй (м)
HOVER_TIME = 20       # секунд зависания (сканирование)
# =====================

async def run():
    drone = System()

    # Подключение к PX4. Порт 14580 — тот, что слушает PX4 (проверено через ss).
    # Формат "udp://host:port" в старом API = подключение как клиент.
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("Ожидание подключения к PX4...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[OK] PX4 подключён!")
            break

    print("Ожидание готовности (GPS/EKF)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[OK] Готов к взлёту!")
            break

    # 1. ВЗЛЁТ
    print(f"[1/3] Взлёт на {ALT_REL} м...")
    await drone.action.set_takeoff_altitude(ALT_REL)
    await drone.action.arm()
    await asyncio.sleep(2)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # 2. ЗАВИСАНИЕ / СКАНИРОВАНИЕ
    print(f"[2/3] Зависание {HOVER_TIME} сек (сканирование)...")
    await asyncio.sleep(HOVER_TIME)

    # 3. ПОСАДКА
    print("[3/3] Посадка на стартовую площадку...")
    await drone.action.land()
    await asyncio.sleep(20)
    print("=== DONE ===")

if __name__ == "__main__":
    asyncio.run(run())
