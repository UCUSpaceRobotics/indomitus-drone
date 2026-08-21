"""Main for drone autonomy - Outdoor GPS Test."""

import time
import multiprocessing
import queue
import sys

from src.comm.mavlink_node import comm_process_loop, create_command

def main():
    print("🚀 [MAIN] Ініціалізація автономної системи (Тест на вулиці з GPS)...")

    # Створюємо черги для спілкування між процесами
    telemetry_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()

    # 1. Запуск фонового процесу MAVLink
    comm_process = multiprocessing.Process(
        target=comm_process_loop,
        args=(telemetry_queue, command_queue, "/dev/ttyAMA0", 921600),
        daemon=True
    )
    comm_process.start()

    print("⏳ [MAIN] Очікування підключення до Pixhawk та ініціалізації EKF...")
    time.sleep(6) # Трохи збільшений час для стабілізації потоків даних при старті

    try:
        # 2. Перехід у LOITER для надійного армінгу
        print("\n>>> КРОК 1: Перехід у режим LOITER...")
        command_queue.put(create_command("set_mode", mode="LOITER"))
        time.sleep(3)

        # 3. Армінг (Запуск моторів)
        print("\n>>> КРОК 2: Запит на ARMING (Запуск моторів)...")
        command_queue.put(create_command("arm", state=True))
        time.sleep(4)

        # 3.5. Перехід у GUIDED ВЖЕ ПІСЛЯ армінгу
        print("\n>>> КРОК 2.5: Перехід у режим GUIDED...")
        command_queue.put(create_command("set_mode", mode="GUIDED"))
        time.sleep(2)

        # 4. Зліт (Тестовий)
        TARGET_ALTITUDE = 2.0
        print(f"\n>>> КРОК 3: Команда TAKEOFF (Зліт на {TARGET_ALTITUDE} метри)...")
        command_queue.put(create_command("takeoff", altitude=TARGET_ALTITUDE))

        # 4.5. Очікування досягнення цільової висоти (до 20 секунд)
        ALTITUDE_REACH_TIMEOUT = 15.0
        reached_target_altitude = False
        print(f"\n>>> КРОК 3.5: Очікування досягнення {TARGET_ALTITUDE}м (таймаут {ALTITUDE_REACH_TIMEOUT}с)...")
        reach_start_time = time.time()
        while (time.time() - reach_start_time) < ALTITUDE_REACH_TIMEOUT:
            try:
                command_queue.put(create_command("send_landing_target", target=(2.0, 0.0, TARGET_ALTITUDE), initiate_landing=False))
                
                telem = telemetry_queue.get(timeout=0.1)
                alt_m = -telem.get('pos_z_m', 0.0)
                mode = telem.get('mode', 'UNKNOWN')
                armed = "ТАК" if telem.get('armed') else "НІ"
                batt = telem.get('battery_voltage_v', 0.0)

                sys.stdout.write(f"\r[НАБІР ВИСОТИ] Режим: {mode:^8} | Арм: {armed:^3} | Висота: {alt_m:>5.2f}м | Батарея: {batt:>5.1f}V ")
                sys.stdout.flush()

                if alt_m >= TARGET_ALTITUDE * 0.9:  # Дозволяємо невелике відхилення
                    reached_target_altitude = True
                    break
            except queue.Empty:
                pass
            time.sleep(0.2)

        print()
        if not reached_target_altitude:
            print(f"\n>>> КРОК 4: Висота {TARGET_ALTITUDE}м не досягнута за {ALTITUDE_REACH_TIMEOUT}с. LAND + DISARM...")
            command_queue.put(create_command("set_mode", mode="LAND"))
            time.sleep(4)
            command_queue.put(create_command("arm", state=False))
            time.sleep(2)
            return

        # 5. Моніторинг польоту / висіння у повітрі
        FLIGHT_DURATION = 7.0  # Час висіння у секундах
        print(f"\n>>> КРОК 4: Моніторинг телеметрії у польоті ({FLIGHT_DURATION} секунд)...")
        start_time = time.time()

        while (time.time() - start_time) < FLIGHT_DURATION:
            try:
                # Дістаємо найсвіжіший словник телеметрії
                telem = telemetry_queue.get(timeout=0.1)

                # Z-координата в NED йде вниз від точки старту, інвертуємо для реальної висоти
                alt_m = -telem.get('pos_z_m', 0.0)
                mode = telem.get('mode', 'UNKNOWN')
                armed = "ТАК" if telem.get('armed') else "НІ"
                batt = telem.get('battery_voltage_v', 0.0)

                # Друк телеметрії в один рядок
                sys.stdout.write(f"\r[ТЕЛЕМЕТРІЯ] Режим: {mode:^8} | Арм: {armed:^3} | Висота: {alt_m:>5.2f}м | Батарея: {batt:>5.1f}V ")
                sys.stdout.flush()

            except queue.Empty:
                pass

            time.sleep(0.2) # Оновлення 5 разів на секунду

        # # 6.5 Рух 1м вперед (по осі X у локальній системі координат)
        # print("\n>>> КРОК 6: Рух на 1 метр вперед (по осі X у локальній системі координат)...")
        # command_queue.put(create_command("move_local_pos", dx=1.0, dy=0.0, dz=0.0))
        # time.sleep(3) # Час на виконання руху

        # # 7. Моніторинг польоту / висіння у повітрі
        # FLIGHT_DURATION = 3.0  # Час висіння у секундах
        # print(f"\n>>> КРОК 7: Моніторинг телеметрії у польоті ({FLIGHT_DURATION} секунд)...")
        # start_time = time.time()

        # while (time.time() - start_time) < FLIGHT_DURATION:
        #     try:
        #         # Дістаємо найсвіжіший словник телеметрії
        #         telem = telemetry_queue.get(timeout=0.1)

        #         # Z-координата в NED йде вниз від точки старту, інвертуємо для реальної висоти
        #         alt_m = -telem.get('pos_z_m', 0.0)
        #         mode = telem.get('mode', 'UNKNOWN')
        #         armed = "ТАК" if telem.get('armed') else "НІ"
        #         batt = telem.get('battery_voltage_v', 0.0)

        #         # Друк телеметрії в один рядок
        #         sys.stdout.write(f"\r[ТЕЛЕМЕТРІЯ] Режим: {mode:^8} | Арм: {armed:^3} | Висота: {alt_m:>5.2f}м | Батарея: {batt:>5.1f}V ")
        #         sys.stdout.flush()

        #     except queue.Empty:
        #         pass

        #     time.sleep(0.2) # Оновлення 5 разів на секунду

        # print() # Перенесення рядка після завершення циклу

        # # 8. Рух 1м вправо (по осі X у локальній системі координат)
        # print("\n>>> КРОК 8: Рух на 1 метр вправо (по осі X у локальній системі координат)...")
        # command_queue.put(create_command("move_local_pos", dx=0.0, dy=1.0, dz=0.0))
        # time.sleep(3) # Час на виконання руху

        # # 8. Моніторинг польоту / висіння у повітрі
        # FLIGHT_DURATION = 3.0  # Час висіння у секундах
        # print(f"\n>>> КРОК 8: Моніторинг телеметрії у польоті ({FLIGHT_DURATION} секунд)...")
        # start_time = time.time()

        # while (time.time() - start_time) < FLIGHT_DURATION:
        #     try:
        #         # Дістаємо найсвіжіший словник телеметрії
        #         telem = telemetry_queue.get(timeout=0.1)

        #         # Z-координата в NED йде вниз від точки старту, інвертуємо для реальної висоти
        #         alt_m = -telem.get('pos_z_m', 0.0)
        #         mode = telem.get('mode', 'UNKNOWN')
        #         armed = "ТАК" if telem.get('armed') else "НІ"
        #         batt = telem.get('battery_voltage_v', 0.0)

        #         # Друк телеметрії в один рядок
        #         sys.stdout.write(f"\r[ТЕЛЕМЕТРІЯ] Режим: {mode:^8} | Арм: {armed:^3} | Висота: {alt_m:>5.2f}м | Батарея: {batt:>5.1f}V ")
        #         sys.stdout.flush()

        #     except queue.Empty:
        #         pass

        #     time.sleep(0.2) # Оновлення 5 разів на секунду

        # print() # Перенесення рядка після завершення циклу

        # 8. Автоматична посадка
        print("\n>>> КРОК 8: Виконання місії завершено. Команда LAND (Посадка)...")
        command_queue.put(create_command("set_mode", mode="LAND"))
        time.sleep(4)

    except KeyboardInterrupt:
        print("\n\n🛑 [MAIN] Аварійне переривання оператором (Ctrl+C)! Перехід у LAND...")
        command_queue.put(create_command("set_mode", mode="LAND"))
        time.sleep(1)

    finally:
        print("\n🔌 [MAIN] Зупинка фонових процесів зв'язку...")
        comm_process.terminate()
        comm_process.join()
        print("✅ [MAIN] Тестування завершено. Система офлайн.")

if __name__ == '__main__':
    main()