# Запуск CartPole DDPG на Raspberry Pi

---

### Что нужно

- Raspberry Pi 5 (ARM64) с Raspberry Pi OS / Debian
- Физический CartPole с прошитым ESP32
- USB-UART кабель между ESP32 и Pi
- Репозиторий на ветке `ddpg-simba`
- Чекпоинт:

```text
outputs/ddpg_simba_small_push_seed42/ddpg_model.pt
```

NB! Сейчас в репозитории файла с моделью нет. Его нужно скачать вот отсюда и положить в `outputs/ddpg_simba_small_push_seed42/` относительно корня репозитория. Скоро будет настроено автоматическое скачивание модели при сборке образа, но пока нужно сделать вручную.

### 1. Установить Docker (если еще не установлен)

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
# выйдите из сессии и зайдите снова
```

Проверка:

```bash
docker --version
docker compose version
groups   # должны быть docker и dialout
```

### 2. Клонировать проект

```bash
git clone https://github.com/robotics-laboratory/cart-pole.git
cd cart-pole
git checkout ddpg-simba
ls -lh outputs/ddpg_simba_small_push_seed42/ddpg_model.pt
```

### 3. Настроить serial

Сейчас надо указать порт, через который подключен ESP32. Его можно найти командой:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Обычно порт будет `/dev/ttyUSB0`

Теперь настраиваем соответствующие переменные окружения в `.env`:

```bash
cp .env.example .env
# укажите стабильный путь адаптера в .env
# SERIAL_PORT=<то что вы получили командой выше>
# SERIAL_SPEED=500000
```

### 4. Собрать окружение

```bash
docker compose build
```

Образ сам ставит зависимости Python 3.10, скачивает Nanopb и генерирует protobuf.

### 5. Обязательный dry-run (без моторов)

```bash
docker compose run --rm cartpole dry-run
```

Ожидаемый результат: модель загрузилась, моторы не трогались.

```bash
docker compose run --rm cartpole doctor
```

### 6. Запуск на железе

Важно! Сейчас запуск стоит на 30 минут, по истечении этого времени нужно будет просто перезапустить той же командой

```bash
docker compose run --rm cartpole hardware
```

Скрипт откроет serial, сделает reset/homing, прогонит один эпизод и сохранит лог в `outputs/ddpg_phys_eval/`.

### 7. Обучение (опционально, CPU)

```bash
docker compose run --rm cartpole train --n-episodes 2 --out-dir outputs/smoke_train
```

### Частые ошибки

| Симптом | Что сделать |
|---------|-------------|
| Нет `SERIAL_PORT` | Проверьте кабель и `.env` |
| Нет прав на serial | Группа `dialout`, перелогин |
| Нет checkpoint | Положите `ddpg_model.pt` в `outputs/ddpg_simba_small_push_seed42/` |
| Ошибки protobuf | `doctor` / пересборка образа |
| Проблемы с системным Python 3.13 | Используйте только Docker |

---

## Default hardware command (reference)

```bash
export SERIAL_PORT=/dev/ttyUSB0
export SERIAL_SPEED=500000

python scripts/ddpg_phys_eval/eval_one_episode.py \
  --checkpoint outputs/ddpg_simba_small_push_seed42/ddpg_model.pt \
  --serial-port "$SERIAL_PORT" \
  --serial-speed "$SERIAL_SPEED" \
  --theta-offset 3.141592653589793 \
  --out-dir outputs/ddpg_phys_eval
```

Inside Docker this is exactly what `./scripts/run_pi.sh hardware` runs.
